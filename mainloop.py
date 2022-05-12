#!/usr/bin/env python3
"""
This script makes MiRo look for a blue ball and kick it
The code was tested for Python 2 and 3
For Python 2 you might need to change the shebang line to

#!/usr/bin/env python
"""
# Imports
##########################
import os
from math import radians
from re import X
from turtle import forward  # This is used to reset the head pose
import numpy as np  # Numerical Analysis library
import cv2  # Computer Vision library

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
    SLOW = 0.1  # Radial speed when turning on the spot (rad/s)
    FAST = 0.4  # Linear speed when kicking the ball (m/s)
    DEBUG = False  # Set to True to enable debug views of the cameras
    ##NOTE The following option is relevant in MiRoCODE
    NODE_EXISTS = False  # Disables (True) / Enables (False) rospy.init_node

    def reset_head_pose(self):
        """
        Reset MiRo head to default position, to avoid having to deal with tilted frames
        """
        self.kin_joints = JointState()  # Prepare the empty message
        self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
        self.kin_joints.position = [0.0, radians(34.0), 0.0, 0.0]
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
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            # Store image as class attribute for further use
            self.input_camera[index] = image
            # Get image dimensions
            self.frame_height, self.frame_width, channels = image.shape
            self.x_centre = self.frame_width / 2.0
            self.y_centre = self.frame_height / 2.0
            # Raise the flag: A new frame is available for processing
            self.new_frame[index] = True
        except CvBridgeError as e:
            # Ignore corrupted frames
            pass
    def callback_sonar(self, ros_range):
        self.range = ros_range.range


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

    def object_detection(self):
        self.yes=True

    def move_back(self):
        print("Moving back")
        self.drive(-0.2,-0.2)
        for _ in range(40):
            rospy.sleep(self.TICK)
        self.drive(0,0)

    def radar_search(self):
        print("Radar searching")
        self.drive(0,0)

        self.kin_joints.position = [0.0,  radians(-8.0), 0.0,  radians(8.0)]
        self.pub_kin.publish(self.kin_joints)

        for _ in range(40):
            rospy.sleep(self.TICK)
            print('sleep1')
        for i in range(0, 55, 5):
            self.kin_joints.position = [0.0,  radians(8.0),  radians(i),  radians(8.0)]
            self.pub_kin.publish(self.kin_joints)
            print('sleep2')
            for _ in range(20):
                rospy.sleep(self.TICK)
            if self.range > 0.2:
                direction = 0 
                print('break1')
                break
            elif i == 55:
                self.kin_joints.position = [0.0, radians(8.0), 0.0, radians(8.0)]
                self.pub_kin.publish(self.kin_joints)
                for _ in range(40):
                    rospy.sleep(self.TICK)
                    print('sleep3')

                for i in range(0, 55, 5):
                    self.kin_joints.position = [0.0, radians(8.0), radians(-i), radians(8.0)]
                    self.pub_kin.publish(self.kin_joints)
                    for _ in range(20):
                        rospy.sleep(self.TICK)
                        print('sleep4')
                    if self.range > 0.2:
                        direction = 1
                        print('break2')
                        break
        if direction == 0:
            self.drive(0.2,-0.2)
            for _ in range(40):
                rospy.sleep(self.TICK)
            self.drive(0,0)                 
        if direction == 1:
            self.drive(-0.2,0.2)
            for _ in range(40):
                rospy.sleep(self.TICK)
            self.drive(0,0)
    
    def forward(self):
        self.drive(0.4,0.4)

    def loop(self):
        """
        Main control loop
        """
        print("MiRo is walking forward, press CTRL+C to halt...")
        # Main control loop iteration counter
        self.counter = 0
        # This switch loops through MiRo behaviours:
        # Find ball, lock on to the ball and kick ball
        while not rospy.core.is_shutdown():

            if self.range < 0.1:
                print('move_nack')
                self.move_back()
            
            # Detect range
            elif self.range < 0.5:
                self.radar_search()
            
            # orient and move forward
            elif self.range >= 0.5:
                print('move forward')
                self.forward()

            # Yield
            self.counter += 1
            rospy.sleep(self.TICK)

if __name__ == "__main__":
    main = MiRoClient()  # Instantiate class
    main.loop()  # Run the main control loop