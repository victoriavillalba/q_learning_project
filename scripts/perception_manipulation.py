#!/usr/bin/env python3

import rospy, numpy, cv2, cv_bridge, moveit_commander

from sensor_msgs.msg import Image

from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from q_learning_project.msg import RobotMoveDBToBlock

import time

from tf.transformations import quaternion_from_euler, euler_from_quaternion

#import keras_ocr


class RobotPerceptionAndManipulation(object):
    def __init__(self):
        
        # initialize node
        #rospy.init_node('perception_movement')

        # innitialize the action criteria for the robot
        robot_action = RobotMoveDBToBlock()
        self.robot_db = robot_action.robot_db
        self.goal_block_num = robot_action.block_id

        # set up ROS / cv bridge
        self.bridge = cv_bridge.CvBridge()

        # ROS subscribe to the topic publishing actions for the robot to take
        rospy.Subscriber("/q_learning/robot_action", RobotMoveDBToBlock, self.do_action)

        # ROS subscribe to robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.twist = Twist()

        # initialize this node
        rospy.init_node('turtlebot3_dance')

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
#        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
#        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

    def do_action(self):
        # get moving info
        robot_action = RobotMoveDBToBlock()
        #robot_action.robot_db = str(action[0]) # "red" = RED, "green" = GREEN, "blue" = BLUE
        #robot_action.block_id = action[1]

        # assign dumbbell action to robot
        self.robot_db = robot_action.robot_db
        self.goal_block_num = rbot_action.block_id


#    def pick_up(self):
 #       arm_joint_goal = [0.0,0.0,0.0,0.0]
  #      gripper_joint_goal = [0.0,0.0]
   #     self.move_group_arm.go(arm_joint_goal, wait=True)
    #    self.move_group_gripper.go(gripper_joint_goal, wait=True)
     #   self.move_group_gripper.stop()
      #  self.move_group_arm.stop()


#    def put_down(self):
 #       arm_joint_goal = [0.0,0.0,0.0,0.0]
  #      gripper_joint_goal = [0.0,0.0]
   #     self.move_group_arm.go(arm_joint_goal, wait=True)
    #    self.move_group_gripper.go(gripper_joint_goal, wait=True)
     #   self.move_group_gripper.stop()
      #  self.move_group_arm.stop()


    def image_callback(self,msg):
        # converts the incoming ROS message to cv2 format and HSV (hue, saturation, value)
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # define the upper and lower bounds depending on the color of the dumbbell from self.robot_db
        lower_color = numpy.array([])
        upper_color = numpy.array([])

        if self.robot_db == "red":
            lower_color = numpy.array([ 200, 0, 0])
            upper_color = numpy.array([255, 50, 50])
        elif self.robot_db == "green":
            lower_color = numpy.array([ 0, 200, 0])
            upper_color = numpy.array([50, 255, 50])
        else:
            lower_color = numpy.array([ 0, 0, 200])
            upper_color = numpy.array([50, 50, 255])
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # we now erase all pixels that aren't the designated color
        h, w, d = image.shape
        search_top = int(3*h/4)
        search_bot = int(3*h/4 + 20)
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        # using moments() function, determine the center of the pixels
        M = cv2.moments(mask)
        # if there are any pixels found
        if M['m00'] > 0:
            if data.ranges[0] <= 0.1:
               # stop the robot if it's close enough to the dumbbell
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
            else:
                # determine the center of the pixels in the image
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                # visualize a yellow circle in our debugging window to indicate
                # the center point of the yellow pixels
                cv2.circle(image, (cx, cy), 20, (255,255,0), -1)

                # proportional control to have the robot follow the pixels
                err = w/2 - cx
                k_p = 1.0 / 100.0
                self.twist.linear.x = 0.2
                self.twist.angular.z = k_p * err
                self.cmd_vel_pub.publish(self.twist)
        else:
            self.twist.angular.z = 0.2
            self.cmd_vel_pub.publish(self.twist)


        # use robot arm
        #self.pick_up()

        # find blocks and go towards them

        ### image code for block perception from the project specs

        #pipeline = keras_ocr.pipeline.Pipeline()

        ### Once you have the pipeline, you can use it to recognize characters,

        ### images is a list of images in the cv2 format
        #images = [img1, img2, ...]

        ### call the recognizer on the list of images
        #prediction_groups = pipline.recognize(images)


        # use robot arm
        #self.put_down()
#    def run(self):
 #       rospy.spin()





if __name__=="__main__":

    rospy.init_node('perception_movement')
    robotrm = RobotPerceptionAndManipulation()
    rospy.spin()

