#!/usr/bin/env python3

import rospy, numpy

from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from q_learning_project.msg import RobotMoveDBToBlock

import time

from tf.transformations import quaternion_from_euler, euler_from_quaternion


class RobotAction(object):

    def __init__(self, robot_dumbbell, goal_block_num):
        self.robot_db = robot_dumbbell
        self.goal_block_num = goal_block_num

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
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")


    def __str__(self):
        output_str = ("Robot action: move " + self.robot_db.upper() + 
                      " to block " + str(self.goal_block_num))
        return output_str

    
    def do_action(self):
        # get moving info
        robot_action = RobotMoveDBToBlock()
        #robot_action.robot_db = str(action[0]) # "red" = RED, "green" = GREEN, "blue" = BLUE
        #robot_action.block_id = action[1]

        # assign dumbbell action to robot
        self.robot_db = robot_action.robot_db
        self.goal_block_num = rbot_action.block_id


    def pick_up(self):
        continue


    def put_down(self):
        continue


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

        pick__up()

        # find blocks and go towards them

        put_down()


class PhantomRobotMovement(object):

    def __init__(self):

        # initialize this node
        rospy.init_node('turtlebot3_phantom_movement')

        # ROS subscribe to the topic publishing actions for the robot to take
        rospy.Subscriber("/q_learning/robot_action", RobotMoveDBToBlock, self.prepare_to_take_robot_action)

        # ROS subscribe to the Gazebo topic publishing the locations of the models
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_received)

        # ROS publish to the Gazebo topic to set the locations of the models
        self.model_states_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)

        # information about the robot action to take
        self.robot_action_queue = []

        # numbered block model names
        self.numbered_block_model_names = {
            1: "number1",
            2: "number2",
            3: "number3"
        }


        self.numbered_block_side_length = 0.8

        # db model names
        self.db_model_names = {
            "red": "robot_dumbbell_red",
            "green": "robot_dumbbell_green",
            "blue": "robot_dumbbell_blue"
        }

        self.current_db_locations = {}

        quat_orientation_of_dbs_list = quaternion_from_euler(1.5708, 0.0, 0.0)
        self.quat_orientation_of_dbs = Quaternion()
        self.quat_orientation_of_dbs.x = quat_orientation_of_dbs_list[0]
        self.quat_orientation_of_dbs.y = quat_orientation_of_dbs_list[1]
        self.quat_orientation_of_dbs.z = quat_orientation_of_dbs_list[2]
        self.quat_orientation_of_dbs.w = quat_orientation_of_dbs_list[3]


    def execute_robot_action(self):

        time.sleep(0.5)

        if (len(self.robot_action_queue) > 0):

            robot_action_to_take = self.robot_action_queue[0]

            pt = Point(
                x=(self.current_numbered_block_locations[robot_action_to_take.goal_block_num].x + 0.6),
                y=self.current_numbered_block_locations[robot_action_to_take.goal_block_num].y,
                z=self.current_numbered_block_locations[robot_action_to_take.goal_block_num].z
            )

            print(robot_action_to_take)

            p = Pose(position=pt,
                     orientation=self.quat_orientation_of_dbs)
            t = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,0))
            m_name = self.db_model_names[robot_action_to_take.robot_db]

            robot_db_model_state = ModelState(model_name=m_name, pose=p, twist=t)

            self.model_states_pub.publish(robot_db_model_state)

            # reset the flag and the action in the queue
            self.robot_action_queue.pop(0)


    def model_states_received(self, data):

        # if we have a robot action in our queue, get the locations of the 
        # dbs and numbered blocks
        if (len(self.robot_action_queue) > 0):

            # get the numbered block locations
            for block_id in self.numbered_block_model_names:
                block_idx = data.name.index(self.numbered_block_model_names[block_id])
                self.current_numbered_block_locations[block_id] = data.pose[block_idx].position

            # get the db locations
            for robot_db in self.db_model_names:
                db_idx = data.name.index(self.db_model_names[robot_db])
                self.current_db_locations[robot_db] = data.pose[db_idx].position

            self.execute_robot_action()



    def prepare_to_take_robot_action(self, data):
        # print(data)
        self.robot_action_queue.append(RobotAction(data.robot_db, data.block_id))


    def run(self):
        rospy.spin()







if __name__=="__main__":

    node = PhantomRobotMovement()
    node.run()
