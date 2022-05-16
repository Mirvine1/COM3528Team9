#!/usr/bin/env python3
"""
This script has been adapted from and heavily inspired by the kick_blue_ball.py script made for
module COM3528 by T.Prescott, A. Lucas et al for the University of Sheffield. Produced for assessment
for Team 9, 2022. The contributors to this script were Matthew Irvine, Henry Wilson and Shuochen Xie.
"""
# Imports
##########################
import os
from math import radians
from re import X
from turtle import forward  # This is used to reset the head pose
import numpy as np  # Numerical Analysis library
import cv2  # Computer Vision library
import time
import rospy  # ROS Python interface
from cv_bridge import CvBridge, CvBridgeError  # ROS -> OpenCV converter
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, UInt32, Int16MultiArray, String
from geometry_msgs.msg import Pose2D, TwistStamped
from sensor_msgs.msg import BatteryState, CompressedImage, Image, Imu, JointState, Range


import miro2 as miro  # Import MiRo Developer Kit library

try:  # For convenience, import this util separately
    from miro2.lib import wheel_speed2cmd_vel  # Python 3
except ImportError:
    from miro2.utils import wheel_speed2cmd_vel  # Python 2
##########################


class MiRoClient:
    """
    Script settings below
    """
    TICK = 0.02  # This is the update interval for the main control loop in secs
    CAM_FREQ = 1  # Number of ticks before camera gets a new frame, increase in case of network lag
    DEBUG = False  # Set to True to enable debug views of the cameras
    ##NOTE The following option is relevant in MiRoCODE
    NODE_EXISTS = False  # Disables (True) / Enables (False) rospy.init_node

    def reset_head_pose(self):
        """
        Reset MiRo head to default position, to avoid having to deal with tilted frames
        """
        self.kin_joints = JointState()  # Prepare the empty message
        self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
        self.kin_joints.position = [0.0,  radians(6.0),  0.0,  radians(8.0)]
        t = 0
        while not rospy.core.is_shutdown():  # Check ROS is running
            # Publish state to neck servos for 1 sec
            self.pub_kin.publish(self.kin_joints)
            rospy.sleep(self.TICK)
            t += self.TICK
            if t > 1:
                break

    def drive(self, speed_l=0.1, speed_r=0.1):  # (m/sec, m/sec)
        """
        Wrapper to simplify driving MiRo by converting wheel speeds to cmd_vel
        """
        # Prepare an empty velocity command message
        msg_cmd_vel = TwistStamped()

        # Desired wheel speed (m/sec)
        wheel_speed = [speed_l, speed_r]

        # Convert wheel speed to command velocity (m/sec, Rad/sec)
        (dr, dtheta) = wheel_speed2cmd_vel(wheel_speed)

        # Update the message with the desired speed
        msg_cmd_vel.twist.linear.x = dr
        msg_cmd_vel.twist.angular.z = dtheta

        # Publish message to control/cmd_vel topic
        self.vel_pub.publish(msg_cmd_vel)

    def callback_caml(self, ros_image):  # Left camera
        self.callback_cam(ros_image, 0)

    def callback_camr(self, ros_image):  # Right camera
        self.callback_cam(ros_image, 1)

    def callback_cam(self, ros_image, index):
        """
        Callback function executed upon image arrival
        """
        # Silently(-ish) handle corrupted JPEG frames
        try:
            # Convert compressed ROS image to raw CV image
            image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "rgb8")
            # Convert from OpenCV's default BGR to RGB
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # Store image as class attribute for further use
            self.input_camera[index] = image
            # Get image dimensions
            self.frame_height, self.frame_width = image.shape
            self.x_centre = self.frame_width / 2.0
            self.y_centre = self.frame_height / 2.0
            # Raise the flag: A new frame is available for processing
            self.new_frame[index] = True
        except CvBridgeError as e:
            # Ignore corrupted frames
            pass
    def callback_sonar(self, ros_range):
        self.range = ros_range.range
    
    def callback_head(self, sensors):
        self.head_touch = sensors
    
    def callback_body(self,sensors):
        self.body_touch = sensors

    def callback_joints(self,sensors):
        self.joints = sensors

    def __init__(self):
        # Initialise a new ROS node to communicate with MiRo
        if not self.NODE_EXISTS:
            rospy.init_node("mainloop", anonymous=True)
        # Give it some time to make sure everything is initialised
        rospy.sleep(2.0)
        # Initialise CV Bridge
        self.image_converter = CvBridge()
        # Individual robot name acts as ROS topic prefix
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        # Create two new subscribers to recieve camera images with attached callbacks
        self.sub_caml = rospy.Subscriber(
            topic_base_name + "/sensors/caml/compressed",
            CompressedImage,
            self.callback_caml,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.sub_camr = rospy.Subscriber(
            topic_base_name + "/sensors/camr/compressed",
            CompressedImage,
            self.callback_camr,
            queue_size=1,
            tcp_nodelay=True,
        )
        # Create new subscriber to recieve sonar data with attached callback
        self.sub_sonar = rospy.Subscriber(
            topic_base_name + "/sensors/sonar",
            Range,
            self.callback_sonar,
            queue_size=1,
            tcp_nodelay=True,
        )
        # Create new subscriber to recieve touch data with attached callback
        self.sub_touch_body = rospy.Subscriber(
            topic_base_name + "/sensors/touch_body",
            UInt16,
            self.callback_body,
            queue_size=1,
            tcp_nodelay=True,
        )
        # Create new subscriber to recieve touch data with attached callback
        self.sub_touch_head = rospy.Subscriber(
            topic_base_name + "/sensors/touch_head",
            UInt16,
            self.callback_head,
            queue_size=1,
            tcp_nodelay=True,
        )
        # Create new subscriber to recieve kinematic data with attached callback
        self.sub_touch_head = rospy.Subscriber(
            topic_base_name + "/sensors/kinematic_joints",
            JointState,
            self.callback_joints,
            queue_size=1,
            tcp_nodelay=True,
        )
        # Create a new publisher to send velocity commands to the robot
        self.vel_pub = rospy.Publisher(
            topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0
        )
        # Create a new publisher to move the robot head
        self.pub_kin = rospy.Publisher(
            topic_base_name + "/control/kinematic_joints", JointState, queue_size=0
        )
        # Create handle to store images
        self.input_camera = [None, None]

        self.edge = [None, None]
        # New frame notification
        self.new_frame = [False, False]
        # Create variable to store a list of ball's x, y, and r values for each camera
        self.ball = [None, None]
        # Set the default frame width (gets updated on reciecing an image)
        self.frame_width = 640
        # Action selector to reduce duplicate printing to the terminal
        self.just_switched = True
        # Bookmark
        self.bookmark = 0
        # Move the head to default pose
        self.reset_head_pose()

        self.tilt = 0
        self.tilt_direc = 'L'
        self.close = 0.05
        self.medium = 0.2
        self.left_pixel = [0,0]
        self.right_pixel = [0,0]

    def move_back(self):
        print("Moving back")
        self.drive(-0.2 + self.tilt/150,-0.2 - self.tilt/150)

    def sonar_search(self):

        left_direction = False
        right_direction = False
        print("Radar searching")
        self.drive(0,0)
        self.kin_joints.position = [0.0,  radians(6.0), 0.0,  radians(8.0)]
        self.pub_kin.publish(self.kin_joints)


        # Block by an obstacle, looking for a way
        for i in range(0, 60, 5):
            self.kin_joints.position = [0.0,  radians(6.0),  radians(i),  radians(8.0)]
            self.pub_kin.publish(self.kin_joints)
            print(i)
            print(self.range)
            print('sleep1')
            time.sleep(0.2)
            print(self.range)
            if self.range > 0.35:
                print(self.range)
                # Find way om the Left 
                left_direction = True
                print('break1')
                break

        for i in range(0, -60, -5):
            self.kin_joints.position = [0.0,  radians(6.0),  radians(i),  radians(8.0)]
            self.pub_kin.publish(self.kin_joints)
            print(i)
            print(self.range)
            print('sleep2')
            time.sleep(0.2)
            print(self.range)
            if self.range > 0.35:
                print(self.range)
                # Find way on the right 
                right_direction = True
                print('break2')
                break

        if left_direction:
                self.drive(-0.2,0.2)
                for _ in range(40):
                    rospy.sleep(self.TICK)
                self.drive(0,0)     

        elif right_direction:
                self.drive(0.2,-0.2)
                for _ in range(40):
                    rospy.sleep(self.TICK)
                self.drive(0,0)

        else:
            # no way on the both sides 
            self.move_back()
    
    def move(self):
        self.drive(0.4,0.4)


    def edge_detect(self, image, index):
        self.new_frame[index] = False
        # load the image, convert it to grayscale, and blur it slightly
        #gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        blurred = cv2.GaussianBlur(image, (5, 5), 0)
        # show the original and blurred images
        #cv2.imshow("Original", image)
        #cv2.imshow("Blurred", blurred)

        # compute a "wide", "mid-range", and "tight" threshold for the edges
        # using the Canny edge detector
        # wide = cv2.Canny(blurred, 10, 200, L2gradient=True)
        # mid = cv2.Canny(blurred, 30, 150, L2gradient=True)
        tight = cv2.Canny(blurred, 240, 250, L2gradient=True)

        # show the output Canny edge maps
        #cv2.imshow("Wide Edge Map", wide)
        #cv2.imshow("Mid Edge Map", mid)
        cv2.imshow("Tight Edge Map", tight)
        #cv2.waitKey(0)
        return tight

    def find_side(self, image):
        y,x = image.shape
        compare_x = x
        for i in range(0,y):
            for j in range(0,x):
                if image[i][j] == 255:
                    if j < compare_x:
                        compare_x = j
                        break
        compare_y = 0
        for i in range(0,y):
            for j in range(0,x):
                if image[i][j] == 255:
                    if j > compare_y:
                        compare_y = j
                        
        compare_y = x - compare_y
        diff = compare_x - compare_y
        print(compare_y)
        print(compare_x)
        if diff < 0:
            direction = 'R'
        else:
            direction = 'L'
        return direction

    def edge_search(self):
        self.kin_joints.position = [0.0, radians(34.0), 0.0, 0.0]
        self.pub_kin.publish(self.kin_joints)
        rospy.sleep(self.TICK*50)
        for index in range(2):  # For each camera (0 = left, 1 = right)
            # Skip if there's no new image, in case the network is choking
            if not self.new_frame[index]:
                continue
            image = self.input_camera[index]
            # Run the detect ball procedure
            self.edge[index] = self.edge_detect(image, index)
        if self.find_side(self.edge[0]) == 'R' and self.find_side(self.edge[1]) == 'R':
            self.drive(0.2, -0.2)
            for _ in range(50):
                rospy.sleep(self.TICK)  
            print('chose right')

        elif self.find_side(self.edge[0]) == 'L' and self.find_side(self.edge[1]) == 'L':
            self.drive(-0.2, 0.2)
            for _ in range(50):
                rospy.sleep(self.TICK)  
            print('chose left')
        else:
            self.sonar_search()

    def loop(self):
        """
        Main control loop
        """
        print("MiRo is walking forward, press CTRL+C to halt...")

        # This switch loops through MiRo behaviours:
        # 
        while not rospy.core.is_shutdown():
            
            # Move back from close objects
            if self.range < self.close:
                print('move back')
                self.move_back()
            
            # Scan for a way round objects in medium range
            elif self.range < self.medium:
                #self.sonar_search()
                self.edge_search()
            
            # Perform typical movement and basic area scanning
            elif self.range >= self.medium:
                self.move()
                self.kin_joints.position = [0.0,  radians(6.0),  radians(self.tilt),  radians(8.0)]
                self.pub_kin.publish(self.kin_joints)
                if self.tilt >= 40:
                    self.tilt_direc = 'R'
                if self.tilt <= -40:
                    self.tilt_direc = 'L' 
                if self.tilt_direc =='R':
                    self.tilt -=5
                if self.tilt_direc =='L':
                    self.tilt +=5
                rospy.sleep(self.TICK*3)


            # Incase not receiving valid range values
            rospy.sleep(self.TICK)

if __name__ == "__main__":
    main = MiRoClient()  # Instantiate class
    main.loop()  # Run the main control loop