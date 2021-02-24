#!/usr/bin/env python3

import rospy 
from q_learning_project.msg import QMatrix, QMatrixRow, QLearningReward, RobotMoveDBToBlock
from collections import Counter
import numpy as np

# Macros for colors 
RED = 1 
GREEN = 2
BLUE = 3

class QLearning(object): 
    def __init__(self): 
        # Initialize this node 
        rospy.init_node('q_learning')
        
        # Subscribe to the reward topic 
        rospy.Subscriber("q_learning/reward", QLearningReward, self.reward_callback)        
        
        # Set publishers for later use 
        self.q_matrix_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=1)
        self.robot_action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveDBToBlock, queue_size=1)
        
        # Waiting for the publisher to be attached
        rospy.sleep(3)
        
        # Initialize Q Matrix
        self.Q = QMatrix()
        self.Q.q_matrix = np.full((64, 9), 0)
        
        # Initialize action Matrix 
        self.A = np.full((64, 64), -1)
        self.initialize_action_matrix()
        
        # Make the initial publish
        self.q_matrix_pub.publish(serialize_Q(self.Q)) 
        
        # Initialize state to s0
        self.state = 0 
        
        # Running window to keep track of recent q_matrix updates 
        self.running_update_window = []
        
        # The time the last reward was received. Used to catch on bogus reward values from the world resetting
        self.time_last_reward = 0
        
        # Time delay between receiving a reward and acting on it. Should have a non-zero value for performance issues
        self.delay = 0.5
        
        # Used to keep track of the last n q_matrix update values
        # The algorithm will determine if it's converged if the last n values did not produce a new update 
        self.running_update_window_len = 200 
        
        # No action was taken yet 
        self.action_choice = -1
        
        # Start the Q-Learning process
        self.sample_action(init=True)
        
        
    

    def get_state_from_row(self, row: int):
        """Given a row, translates it to state information

        Args:
            row (int): 0-64 
            
        Returns:
            (tuple): (row, red loc, green loc, blue loc) # 0 = origin, 1/2/3 = blocks
        """       
        index = [0,1,2,3]
        blue_location = index[row // 16]
        green_location = index[(row % 16) // 4]
        red_location = index[row % 4 ]
        return (row, red_location, green_location, blue_location)
        
        
    def get_col_from_action(self, action: tuple):
        """Gets action number column from a given action tuple 

        Args:
            action (tuple): (move_dumbell_color, to_block_number) e.g. (RED/GREEN/BLUE, 1/2/3)

        Returns:
            int: column number of action
        """        
        dumbbell_color = action[0] # RED/GREEN/BLUE are constants corresponding to 1, 2, 3
        to_block = action[1]
        return (dumbbell_color - 1) * 3 + (to_block - 1)
    
    
    def get_action_from_col(self, col):
        """Given an action column, returns a tuple of what the action is

        Args:
            col (int): action column number

        Returns:
            tuple<int, int>: (dumbbell_color, to_block)
        """        
        dumbbell_color = (col // 3) + 1
        to_block = (col % 3) + 1
        return (dumbbell_color, to_block)
        
                
    def initialize_action_matrix(self): 
        """Initializes the action matrix for valid/invalid actions
        """        
        for start_i, cols in enumerate(self.A):
            for next_j, value in enumerate(cols): 
                start_state = self.get_state_from_row(start_i)
                next_state = self.get_state_from_row(next_j) # (row_number, red loc, blue loc, green loc)
                
                # Valid states include moving exactly 1 dumbbell to a block that's
                # not already occupied, or to origin. 
                num_moves, moves = count_moves_from_states(start_state, next_state)
                next_state_loc_counts = Counter(next_state[1:]) # key: location (where 0 = origin, 1/2/3 = block #), value: count 
                del next_state_loc_counts[0] # ignore origin location
                is_valid_next_state = all(map(lambda x: x <= 1, next_state_loc_counts.values()))  # There must not be more than 1 dumbbell at each block
                if (num_moves == 1 and is_valid_next_state):
                    move = moves[0] # Only 1 move 
                    if (start_state == next_state or move[1] == 0): # doing nothing or moving to origin is not valid
                        continue 
                    self.A[start_i][next_j] = self.get_col_from_action(move)    


    def sample_action(self, init=False):
        print("sampling action")
        # selecting a_t at random 
        current_state = self.state
        if (init):
            current_state = 0
        print("current_state", self.get_state_from_row(current_state))
        possible_actions = np.array(self.A[current_state])
        possible_actions = possible_actions[possible_actions != -1] 
        print("Possible actions", possible_actions)
        if (len(possible_actions) == 0):
            print("no possible action for this state")
            self.action_choice = -2
            return
        
        choice = np.random.choice(possible_actions)
        self.action_choice = choice
        action = self.get_action_from_col(choice)

        robot_action = RobotMoveDBToBlock()
        robot_action.robot_db = convert_color_to_str(action[0]) # "red", "green", "blue"
        robot_action.block_id = action[1] # 1, 2, 3
        self.robot_action_pub.publish(robot_action)
        

    def reward_callback(self, data: QLearningReward):
        print("action", self.action_choice, self.get_action_from_col(self.action_choice))
        print("Reward received", data.reward, data.header.stamp.to_time())
        # Catching bogus reward where reset_world.py sends us a reward when the world resets (this should not always happen but we need to catch it for our code's logic)
        if (data.header.stamp.to_time() - self.time_last_reward < (self.delay / 2)):
            print("Received bogus reward for reset state")
            print("\n")
            return
        self.time_last_reward = data.header.stamp.to_time()
        
        # Sleep to give Gazebo time to recuperate
        rospy.sleep(self.delay)
        
        action_taken = self.action_choice
        # This if statement is never called, but placed here for safe measure
        if (action_taken == -2): # No valid action, i.e. all 3 dumbbells already in front of 3 distinct blocks
            print("Did not take an action, should not've received a reward")
            return
        
        current_state = self.state
        next_state = np.where(np.array(self.A[current_state]) == action_taken)[0][0]
        print("state", current_state, "next state", next_state, self.get_state_from_row(next_state))
        alpha = 1
        gamma = 0.2
        print("best next action", max(self.Q.q_matrix[next_state]), np.where(self.Q.q_matrix[next_state] == max(self.Q.q_matrix[next_state]))[0])
        
        # Applying basic q-learning update formula
        Q_st_at = self.Q.q_matrix[current_state][action_taken]
        updated_q_matrix_val = Q_st_at + alpha * (data.reward + gamma * max(self.Q.q_matrix[next_state]) - Q_st_at)
        self.Q.q_matrix[current_state][action_taken] = updated_q_matrix_val
        print("new q_matrix_val ", Q_st_at, "->", int(updated_q_matrix_val))
        
        # Update the running window tracker
        if (len(self.running_update_window) >= self.running_update_window_len):
            self.running_update_window.pop(0)
        self.running_update_window.append(str(Q_st_at) + "->" + str(updated_q_matrix_val))
        print(self.running_update_window)        
        
        # Publishing new q matrix 
        self.q_matrix_pub.publish(serialize_Q(self.Q))

        # Reset the state since all 3 dumbbells have exactly 1 block in front of them        
        next_state_pos = self.get_state_from_row(next_state)[1:]
        if (len(set(next_state_pos)) == len(next_state_pos) and 0 not in next_state_pos):
            print('resetting', next_state_pos)
            next_state = 0
        
        # Checking if matrix is converged
        if(self.check_matrix_converge()):
            print("Converged, stopping")
            return
        
        # Continue the learning iterations
        self.state = next_state
        print("\n")
        self.sample_action()
        return
    
    
    def check_matrix_converge(self):
        """Checks if the q-learning matrix is converged by counting how many static changes occur

        Returns:
            Boolean: True if the q-matrix is believed to have converged, false o.w.
        """        
        # If all of the running window changes are to the same value, then we stop learning 
        test = list(map(lambda x: int(x.split("->")[0]) == int(float(x.split("->")[1])), self.running_update_window))
        is_converged = np.count_nonzero(test) == self.running_update_window_len
        print("is_coverged ", is_converged, np.count_nonzero(test), "/", self.running_update_window_len)
        return is_converged
        
        
    def run(self):
        rospy.spin()
        
        
""" 
Helper functions 
"""
def count_moves_from_states(state1, state2):
    """Count the number of moves given 2 state tuples 

    Args:
        state1 (tuple): start state 
        state2 (tuple): next state

    Returns:
        (int, array): (number of moves needed to reach next state, list of moves made to transition to next state given as (dumbbell, location))
    """        
    num_moves = 0
    moves = [] # Array of tuples e.g. (RED, 1) - (dumbbell color, move location)
    for i in [RED, GREEN, BLUE]: 
        if (state1[i] != state2[i]):
            num_moves += 1
            moves.append((i, state2[i]))
    return (num_moves, moves)


def convert_color_to_str(color): 
    if color == RED:
        return "red"
    elif color == GREEN: 
        return "green"
    elif color == BLUE:
        return "blue"
    else:
        raise TypeError("color is not one of 1, 2, or 3")


def serialize_Q(Q: np.ndarray):
    """Given a Q Matrix numpy 2d array, serializes it to the right type for ROS

    Args:
        Q (np.ndarray): Q matrix to serialize

    Returns:
        QMatrix: Complete QMatrix object to be published
    """    
    ret = QMatrix()
    ret.q_matrix = [QMatrixRow() for i in range(64)]
    for i in range(64):
        row = []
        for j in range(9):
            row.append(Q.q_matrix[i][j])
        ret.q_matrix[i].q_matrix_row = row
    return ret


if __name__ == "__main__":
    node = QLearning()
    node.run()

