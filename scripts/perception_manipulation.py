#!/usr/bin/env python3

import rospy, numpy, cv2, cv_bridge, moveit_commander

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from q_learning_project.msg import RobotMoveDBToBlock

import time

from tf.transformations import quaternion_from_euler, euler_from_quaternion

#import keras_ocr

# Robot action intermediate states 
NOTHING = 0
MOVE_TO_DB = 1 # includes searching 
PICK_UP_DB = 2
MOVE_TO_BLOCK = 3 # includes searching
DROP_DB = 4

class RobotPerceptionAndManipulation(object):
    def __init__(self):
        
        rospy.init_node('perception_movement')

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
        self.data = LaserScan()
        rospy.Subscriber("scan", LaserScan, self.robot_scan_received)

        # ROS subscribe to the topic publishing actions for the robot to take
        rospy.Subscriber("/q_learning/robot_action", RobotMoveDBToBlock, self.do_action)

        # ROS subscribe to robot's RGB camera data stream
        self.image = Image()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)

        # Movement publisher
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        # Controller node response publishder
        self.controller_pub = rospy.Publisher('/q_learning/controller', RobotMoveDBToBlock, queue_size=1)

        rospy.sleep(3)
        
        print("ready")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

    def do_action(self, robot_action: RobotMoveDBToBlock):
        print("Do action")
        # assign dumbbell action to robot
        self.robot_db = robot_action.robot_db
        self.goal_block_num = robot_action.block_id
        self.action_state = MOVE_TO_DB
        
        print("Assigned task move dumbbell", self.robot_db, "to", self.goal_block_num)

        # set up arm to be ready to pick up a dumbbell
        self.move_group_arm.go([0.0,0.7,0.0,-0.5], wait=True)
        self.move_group_gripper.go([0.1,0.1], wait=True)
        self.move_group_arm.stop()
        self.move_group_gripper.stop()

        print("arm is aligned to pick up")


    def robot_scan_received(self, data: LaserScan):
        self.data = data

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
            self.move_to_block
        elif self.action_state == DROP_DB:
            self.drop_db()
            
    def pick_up_db(self):
        arm_joint_goal = [0.0,-1.0,0.3,0.7]
        gripper_joint_goal = [0.04,0.04]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()
        self.move_group_arm.stop()
        print("TODO!")
    
    def move_to_block(self):
        print("TODO!")
        
    def drop_db(self):
        arm_joint_goal = [0.0,0.7,0.0,-0.5]
        gripper_joint_goal = [0.1,0.1]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()
        self.move_group_arm.stop()
        arm_joint_goal = [0.0,-0.3,1.5,-0.9]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        sel.move_group_arm.stop()
        print("TODO!")
    
        # When DB has been dropped
        completed_action = RobotMoveDBToBlock()
        completed_action.robot_db = self.robot_db
        completed_action.block_id = self.goal_block_num
        self.controller_pub.publish(completed_action)

    def move_to_db(self):
        image = self.image
        
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # define the upper and lower bounds depending on the color of the dumbbell from self.robot_db
        lower_color = numpy.array([])
        upper_color = numpy.array([])

        if self.robot_db == "red":
            lower_color = numpy.array([ 0, 128, 128])
            upper_color = numpy.array([ 5, 255, 255])
        elif self.robot_db == "green":
            lower_color = numpy.array([ 55, 128, 128])
            upper_color = numpy.array([ 65, 255, 255])
        elif self.robot_db == "blue":
            lower_color = numpy.array([ 115, 128, 128])
            upper_color = numpy.array([ 125, 255, 255])
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
            if self.data.ranges[0] <= 0.4:
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
                cv2.circle(image, (cx, cy), 20, (255,255,0), -1)

                # proportional control to have the robot follow the pixels
                err = w/2 - cx
                k_p = 1.0 / 100.0
                self.twist.linear.x = 0.2
                self.twist.angular.z = k_p * err
                self.cmd_vel_pub.publish(self.twist)
        else: 
            print("no pixels found, searching..")
            self.twist.angular.z = 0.2
            self.twist.linear.x = 0.1
            self.cmd_vel_pub.publish(self.twist)
    
    def image_callback(self, msg):
        # converts the incoming ROS message to cv2 format and HSV (hue, saturation, value)
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        self.image = image
        
        
    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.action_loop()
            r.sleep()

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

if __name__=="__main__":

    node = RobotPerceptionAndManipulation()
    node.run()

