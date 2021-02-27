#!/usr/bin/env python3

import rospy 
from std_msgs.msg import Bool
from q_learning_project.msg import QMatrix, QMatrixRow, QLearningReward, RobotMoveDBToBlock
from phantom_robot_movement import PhantomRobotMovement
import numpy as np
class Controller(object): 
    def __init__(self):
        print("controller")
        rospy.init_node("controller")

        self.Q = QMatrix()
        self.load_q_matrix()
        self.state = 0

        self.robot_action_publisher = rospy.Publisher("/q_learning/robot_action", RobotMoveDBToBlock, queue_size=1)
        
        rospy.sleep(3)
        self.selectActionFromState()
        # rospy.Subscriber("q_learning/q_matrix_converged", Bool, self.q_matrix_converged, queue_size=2)
        # rospy.Subscriber("q_learning/q_matrix", QMatrix, self.updateQMatrix, queue_size=2)
    
    def run(self):
        rospy.spin()
    
    def load_q_matrix(self):
        print("loading q matrix")
        self.Q.q_matrix = np.loadtxt("q_matrix_dump.txt")
            
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
    
    def selectActionFromState(self):
        print("Selecting action")
        best_action_reward = max(self.Q.q_matrix[self.state])
        action = np.where(self.Q.q_matrix[self.state] == best_action_reward)[0]
        if len(action) > 1: 
            print("Multiple best actions found, selecting first one", self.state, action)
        else:
            print("Selecting action", self.state, action)
        dumbbell_color, to_block = self.get_action_from_col(action[0])
        robot_action = RobotMoveDBToBlock()
        robot_action.robot_db = convert_color_to_str(dumbbell_color)
        robot_action.block_id = to_block
        print("publishing", robot_action)
        self.robot_action_publisher.publish(robot_action)
        
        
def convert_color_to_str(color): 
    if color == 0:
        return "red"
    elif color == 1: 
        return "green"
    elif color == 2:
        return "blue"
    else:
        raise TypeError("color is not one of 1, 2, or 3")
    
if __name__ == "__main__":
    node = Controller()
    node.run()