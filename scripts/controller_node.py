#!/usr/bin/env python3

import rospy 
from std_msgs.msg import Bool
from q_learning_project.msg import QMatrix, QMatrixRow, QLearningReward, RobotMoveDBToBlock
from phantom_robot_movement import PhantomRobotMovement
import numpy as np

RED = 1
GREEN = 2
BLUE = 3 

class Controller(object): 
    def __init__(self):
        print("controller")
        rospy.init_node("controller")

        self.Q = QMatrix()
        self.load_q_matrix()
        self.state = 0

        self.robot_action_publisher = rospy.Publisher("/q_learning/robot_action", RobotMoveDBToBlock, queue_size=1)
        
        rospy.Subscriber("/q_learning/controller", RobotMoveDBToBlock, self.controller_subscriber_callback)
        
        rospy.sleep(3)
        self.selectActionFromState()
        # rospy.Subscriber("q_learning/q_matrix_converged", Bool, self.q_matrix_converged, queue_size=2)
        # rospy.Subscriber("q_learning/q_matrix", QMatrix, self.updateQMatrix, queue_size=2)
    
    def run(self):
        rospy.spin()
    
    def load_q_matrix(self):
        print("loading q matrix")
        self.Q.q_matrix = np.loadtxt("q_matrix_dump.txt")
        
    def controller_subscriber_callback(self, data: RobotMoveDBToBlock):
        # Callback when perception manipulation module claims that it completed what we told it to do
        # Advance current state, and select another action
        current_state = list(get_state_from_row(self.state))
        moved_db = convert_str_to_color(data.robot_db)
        current_state[moved_db] = data.block_id
        self.state = get_row_from_state(tuple(current_state))
        print("new state", self.state)
        self.selectActionFromState()
        return
        
    def selectActionFromState(self):
        print("Selecting action")
        best_action_reward = max(self.Q.q_matrix[self.state])
        action = np.where(self.Q.q_matrix[self.state] == best_action_reward)[0]
        if len(action) > 1: 
            print("Multiple best actions found, selecting first one", self.state, action)
        else:
            print("Selecting action", self.state, action)
        dumbbell_color, to_block = get_action_from_col(action[0])
        print("dumbbell color", dumbbell_color, "to", to_block)
        robot_action = RobotMoveDBToBlock()
        robot_action.robot_db = convert_color_to_str(dumbbell_color)
        robot_action.block_id = to_block
        print("publishing", robot_action)
        self.robot_action_publisher.publish(robot_action)
        
        
def convert_color_to_str(color): 
    if color == 1:
        return "red"
    elif color == 2: 
        return "green"
    elif color == 3:
        return "blue"
    else:
        raise TypeError("color is not one of 1, 2, or 3")

def convert_str_to_color(s):
    if s == "red":
        return RED 
    elif s == "green":
        return GREEN 
    elif s == "blue":
        return BLUE 
    else:
        raise TypeError("s is not one of 'red', 'green', 'blue'")
    
def get_action_from_col(col):
    """Given an action column, returns a tuple of what the action is

    Args:
        col (int): action column number

    Returns:
        tuple<int, int>: (dumbbell_color, to_block)
    """        
    dumbbell_color = (col // 3) + 1
    to_block = (col % 3) + 1
    return (dumbbell_color, to_block)

def get_state_from_row(row: int):
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

def get_row_from_state(state: tuple):
    """Gets row number from state 

    Args:
        state (tuple): (row, red_loc, green_loc, blue_loc) where `row` is a bogus value
    """    
    row, red_location, green_location, blue_location = state 
    ret = 0
    ret += blue_location * 16
    ret += green_location * 4
    ret += red_location
    return ret
    
    
if __name__ == "__main__":
    node = Controller()
    node.run()