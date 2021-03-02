#!/usr/bin/env python3

import rospy
import numpy
import cv2
import cv_bridge
import moveit_commander

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from q_learning_project.msg import RobotMoveDBToBlock

import time
import moveit_commander
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import keras_ocr
import numpy as np 

# Robot action intermediate states
NOTHING = 0
MOVE_TO_DB = 1  # includes searching
PICK_UP_DB = 2
MOVE_TO_BLOCK = 3  # includes searching
DROP_DB = 4


class RobotPerceptionAndManipulation(object):
    def __init__(self):

        rospy.init_node('perception_movement')

        # Download pretrained keras ocr
        self.pipeline = keras_ocr.pipeline.Pipeline()

        # innitialize the action criteria for the robot
        robot_action = RobotMoveDBToBlock()
        self.robot_db = robot_action.robot_db
        self.goal_block_num = robot_action.block_id

        # initialize state of the robot in performing the action
        self.action_state = NOTHING

        # set up ROS / cv bridge
        self.bridge = cv_bridge.CvBridge()
        self.twist = Twist()

        # LIDAR data
        self.laserdata = LaserScan()
        rospy.Subscriber("scan", LaserScan, self.robot_scan_received)

        # ROS subscribe to the topic publishing actions for the robot to take
        rospy.Subscriber("/q_learning/robot_action",
                         RobotMoveDBToBlock, self.do_action)

        # ROS subscribe to robot's RGB camera data stream
        self.image = Image()
        self.image_sub = rospy.Subscriber(
            'camera/rgb/image_raw', Image, self.image_callback)

        # Movement publisher
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Controller node response publishder
        self.controller_pub = rospy.Publisher(
            '/q_learning/controller', RobotMoveDBToBlock, queue_size=1)

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander(
            "gripper")

        self.move_group_gripper.go([0.019,0.019], wait=True)
        self.move_group_arm.go([0.0,0.95,-0.7,-0.45], wait=True)
        self.move_group_gripper.stop()
        self.move_group_arm.stop()

        rospy.sleep(3)
        self.cmd_vel_pub.publish(Twist())

        print("ready")

    def do_action(self, robot_action: RobotMoveDBToBlock):
        """Callback for receiving instructions on what to do

        Args:
            robot_action (RobotMoveDBToBlock): Action that should be taken
        """
        # assign dumbbell action to robot
        self.robot_db = robot_action.robot_db
        self.goal_block_num = robot_action.block_id
        self.action_state = MOVE_TO_DB

        # set up arm to be ready to pick up a dumbbell
        #self.move_group_arm.go([0.0,1.0,-0.5,-0.5], wait=True)
        #self.move_group_gripper.go([0.05,0.05], wait=True)
        #self.move_group_gripper.stop()
        #self.move_group_arm.stop()
        print("arm is aligned to pick up")

    def robot_scan_received(self, data: LaserScan):
        """Callback for laser scan data 

        Args:
            data (LaserScan): laser scan data
        """
        self.laserdata = data

    def action_loop(self):
        """Dispatches what action we should be taking depending on our intermediate action state 
        """
        if self.action_state == NOTHING:
            self.cmd_vel_pub.publish(Twist())
        elif self.action_state == MOVE_TO_DB:
            self.move_to_db()
        elif self.action_state == PICK_UP_DB:
            self.pick_up_db()
        elif self.action_state == MOVE_TO_BLOCK:
            self.move_to_block()
        elif self.action_state == DROP_DB:
            self.drop_db()

    def pick_up_db(self):
        """Sequence of action to pick up a dumbbell
        """
        print("Picking up db")
        #arm_joint_goal = [0.0,0.8,-0.4,-0.4]
        #gripper_joint_goal = [0.004,0.004]
        #self.move_group_arm.go(arm_joint_goal, wait=True)
        #self.move_group_gripper.go(gripper_joint_goal, wait=True)
        #self.move_group_gripper.stop()
        #self.move_group_arm.stop()
        
        #self.twist.angular.z = 0.1
        #self.cmd_vel_pub.publish(self.twist)
        
        arm_joint_goal = [0.0,0.0,-0.45,-0.1]
        gripper_joint_goal = [0.004, 0.004]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_gripper.stop()
        self.move_group_arm.stop()
        self.action_state = MOVE_TO_BLOCK

    def move_to_block(self):
        """Sequence of actions to identify and move to a block
        """
        print("moving to block")
        block_target = self.goal_block_num
        interval = len(self.laserdata.ranges) // 10
        front = min(
            self.laserdata.ranges[interval * -1:] + self.laserdata.ranges[:interval])

        # Check if top image contains a black color number
        lower_color = numpy.array([0, 0, 0])
        upper_color = numpy.array([255, 255, 0])
        mask = cv2.inRange(cv2.cvtColor(
            self.image, cv2.COLOR_BGR2HSV), lower_color, upper_color)
        mask[self.image.shape[0]//2:self.image.shape[0],
             0:self.image.shape[1]] = 0  # Ignore bottom half i.e. shadows
        # A distance >= 1 is necessary for image recognition (or else we're too close)
        if (front <= 1.2 and 255 in mask):
            if (front > 0.4):
                # Move close to block
                twist = Twist()
                twist.linear.x = 0.1
                twist.angular.z = 0
                self.cmd_vel_pub.publish(twist)
            else:
                # Stop moving
                self.cmd_vel_pub.publish(Twist())
                self.action_state = DROP_DB
        else:
            # Look for the block
            # Stop the robot before running image recognition
            self.cmd_vel_pub.publish(Twist())
            prediction_groups = self.pipeline.recognize([self.image])

            print(prediction_groups)

            block_target_chars = [str(block_target)]
            if (block_target == 1):
                block_target_chars.append('l')
            elif (block_target == 3):
                # likely confused targets
                block_target_chars.append('5')
                block_target_chars.append('b')
                block_target_chars.append('s')
                block_target_chars.append('31')

            matches_predictions = [prediction_groups[0][i] for i in range(len(prediction_groups[0])) if prediction_groups[0][i][0] in block_target_chars]
            if (len(matches_predictions) > 1):
                # 2 matches, check lidar data to pick 
                print("2 matches found")
                # Determine if looking at right or left block given that we're looking from near the centre
                centroids = [(sum([p[0] for p in box[1]]) / len(box[1]), sum(p[1] for p in box[1]) / len(box[1])) for box in matches_predictions]
                h, w, d = self.image.shape 
                if (self.laserdata.ranges[:len(self.laserdata.ranges)//2].count(np.inf) > self.laserdata.ranges[len(self.laserdata.ranges)//2:].count(np.inf)):
                    # looking at left block, pick leftmost box in predictions 
                    print("looking at left block")
                    bias = -20
                    err = w/2 - (min(centroids)[0] + bias) # min checks for first item in tuple which is the x coord
                    k_p = 1.0 / 800.0
                    self.twist.linear.x = 0.4
                    self.twist.angular.z = k_p * err
                    self.cmd_vel_pub.publish(self.twist)
                    rospy.sleep(1)  # hold event thread for let it move
                else:
                    # looking at right block, pick rightmost box in predictions
                    print("looking at right block")
                    bias = 20
                    err = w/2 - (max(centroids)[0] + bias) # max checks for first item in tuple which is the x coord
                    print('max', max(centroids))
                    k_p = 1.0 / 800.0
                    self.twist.linear.x = 0.4
                    self.twist.angular.z = k_p * err
                    self.cmd_vel_pub.publish(self.twist)
                    rospy.sleep(1)  # hold event thread for let it move
            elif (len(matches_predictions) == 1):
                # 1 exact match 
                print("1 block target", block_target, "found")
                recognized_char_box = matches_predictions[0][1]
                x = [p[0] for p in recognized_char_box]
                y = [p[1] for p in recognized_char_box]
                centroid = (sum(x) / len(recognized_char_box),
                            sum(y) / len(recognized_char_box))

                h, w, d = self.image.shape
                err = w/2 - (centroid[0])
                k_p = 1.0 / 1000.0
                self.twist.linear.x = 0.5
                self.twist.angular.z = k_p * err
                self.cmd_vel_pub.publish(self.twist)
                rospy.sleep(1)  # hold event thread for let it move
            else: 
                # no match
                print("no character found, searching..")
                self.twist.angular.z = 0 
                self.twist.linear.x = -0.05
                self.cmd_vel_pub.publish(self.twist)
                rospy.sleep(1)
                self.twist.angular.z = 0.5
                self.twist.linear.x = 0
                self.cmd_vel_pub.publish(self.twist)
                # Sleep for a little before the next event loop to let it search
                rospy.sleep(1.5)


                

    def drop_db(self):
        """sequence of actions to drop dumbbell at current location
        """
        arm_joint_goal = [0.0,0.95,-0.7,-0.45]
        gripper_joint_goal = [0.019, 0.019]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()
        self.move_group_arm.stop()
        #arm_joint_goal = [0.0, -0.3, 1.5, -0.9]
        #self.move_group_arm.go(arm_joint_goal, wait=True)
        #self.move_group_arm.stop()

        # When DB has been dropped

        # Should move back a little bit to avoid bumping into things
        twist = Twist()
        twist.linear.x = -0.5
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(1)
        self.cmd_vel_pub.publish(Twist())

        completed_action = RobotMoveDBToBlock()
        completed_action.robot_db = self.robot_db
        completed_action.block_id = self.goal_block_num
        self.controller_pub.publish(completed_action)
        self.action_state = NOTHING

        return

    def move_to_db(self):
        """sequence of actions to move to a particular dumbbell
        """
        image = self.image

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # define the upper and lower bounds depending on the color of the dumbbell from self.robot_db
        lower_color = numpy.array([])
        upper_color = numpy.array([])

        if self.robot_db == "red":
            lower_color = numpy.array([0, 128, 128])
            upper_color = numpy.array([5, 255, 255])
        elif self.robot_db == "green":
            lower_color = numpy.array([55, 128, 128])
            upper_color = numpy.array([65, 255, 255])
        elif self.robot_db == "blue":
            lower_color = numpy.array([115, 128, 128])
            upper_color = numpy.array([125, 255, 255])
        else:
            # print("no robot_db found, returning for now")
            return

        # Masking only valid colors
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Shape info
        h, w, d = image.shape

        # using moments() function, determine the center of the pixels
        M = cv2.moments(mask)

        if M['m00'] > 0:
            print("pixels found")
            if self.laserdata.ranges[0] <= 0.25:
               # stop the robot if it's close enough to the dumbbell
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)

                # Set new action state
                self.action_state = PICK_UP_DB
            else:
                # determine the center of the pixels in the image
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                # visualize a yellow circle in our debugging window to indicate
                # the center point of the yellow pixels
                cv2.circle(image, (cx, cy), 20, (255, 255, 0), -1)

                # proportional control to have the robot follow the pixels
                err = w/2 - cx
                k_p = 1.0 / 1000.0
                self.twist.linear.x = 0.1
                self.twist.angular.z = k_p * err
                self.cmd_vel_pub.publish(self.twist)
        else:
            print("no pixels found, searching..")
            self.twist.angular.z = 0.2
            self.twist.linear.x = 0
            # self.twist.linear.x = 0.1
            self.cmd_vel_pub.publish(self.twist)

    def image_callback(self, msg):
        """Callback for receiving image data

        Args:
            msg (Image): Camera image
        """
        # converts the incoming ROS message to cv2 format and HSV (hue, saturation, value)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.image = image

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.action_loop()
            r.sleep()


if __name__ == "__main__":
    node = RobotPerceptionAndManipulation()
    node.run()
