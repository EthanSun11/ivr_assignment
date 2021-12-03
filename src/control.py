#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import math
import sympy as sy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
from math import cos, sin

last_red_image1 = [0, 0]
last_red_image2 = [0, 0]


class image_converter:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)
        # initialize a publisher to send robot end-effector position
        self.end_effector_pub = rospy.Publisher("end_effector_prediction", Float64MultiArray, queue_size=10)
        # initialize a publisher to send joints' angular position to the robot
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()
        # record the begining time
        self.time_trajectory = rospy.get_time()
        self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')
        # initialize error and derivative of error for trajectory tracking
        self.error = np.array([0.0, 0.0], dtype='float64')
        self.error_d = np.array([0.0, 0.0], dtype='float64')

        # publish predicted end_effector
        self.x_pub = rospy.Publisher("x", Float64, queue_size=10)
        self.y_pub = rospy.Publisher("y", Float64, queue_size=10)
        self.z_pub = rospy.Publisher("z", Float64, queue_size=10)

        # publish predicted end_effector
        self.fk_pub = rospy.Publisher("fk", Float64MultiArray, queue_size=10)

        # publish target end_effector
        self.xt_pub = rospy.Publisher("xt", Float64, queue_size=10)
        self.yt_pub = rospy.Publisher("yt", Float64, queue_size=10)
        self.zt_pub = rospy.Publisher("zt", Float64, queue_size=10)

        # initialize detected angles
        self.joint1 = 0
        self.joint3 = 0
        self.joint4 = 0

        # initialize end-effector position
        self.end_effector = Float64MultiArray()
        self.end_effector.data = [0, 0, 0]

        self.target = Float64MultiArray()
        self.target.data = [0, 0, 0]

        # hardcoded initial position of green and yellow joints so that i do not calculate ratio every time
        self.pixel2meter_image1 = 0.034777349758716214
        self.pixel2meter_image2 = 0.03598350534333363
        self.yellow_pos = np.array([395, 399, 430])
        self.green_pos = np.array([390, 401, 543])

        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback_image1)
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback_image2)

        # subscribe to the target position
        self.target_sub = rospy.Subscriber("target_pos", Float64MultiArray, self.callback_target)

        # subscribe to detected angles
        self.angle1_sub = rospy.Subscriber("joint_angle_1", Float64, self.callback_angle1)
        self.angle3_sub = rospy.Subscriber("joint_angle_3", Float64, self.callback_angle3)
        self.angle4_sub = rospy.Subscriber("joint_angle_4", Float64, self.callback_angle4)

    # Detecting the centre of the red circle
    def detect_red(self, image):
        # Isolate the blue colour in the image as a binary image
        mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
        # im2=cv2.imshow('mask',mask)
        # cv2.waitKey(1)
        # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        # Obtain the moments of the binary image
        M = cv2.moments(mask)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            if image is self.cv_image1:
                last_red_image1[0] = cx
                last_red_image1[1] = cy
                return np.array([cx, cy])
            else:
                last_red_image2[0] = cx
                last_red_image2[1] = cy
                return np.array([cx, cy])
            # this is in case red is blocked by blue
        else:
            if image is self.cv_image1:
                return np.array(last_red_image1)
            else:
                return np.array(last_red_image2)

    # Detecting the centre of the green circle
    def detect_green(self, image):
        mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        im2 = cv2.imshow('mask', mask)
        cv2.waitKey(1)
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])

    # Detecting the centre of the yellow circle
    def detect_yellow(self, image):
        mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])

    # Calculate the conversion from pixel to meter
    def pixel2meter(self, image):
        # Obtain the centre of each coloured blob
        if image is self.cv_image1:
            circle1Pos = np.array([399, 430])
            circle2Pos = np.array([401, 543])
        else:
            circle1Pos = np.array([395, 430])
            circle2Pos = np.array([390, 543])
        # find the distance between two circles
        dist = np.sum((circle1Pos - circle2Pos) ** 2)
        return 4 / np.sqrt(dist)

    # calculate x,y,z coordinates of end-effector
    def calc_pos(self, pos1, pos2):
        # x = pos2[0] - self.green_pos[0]
        # y = pos1[1] - self.green_pos[1]
        a = self.pixel2meter(self.cv_image1)
        b = self.pixel2meter(self.cv_image2)
        x = (pos2[0] - self.green_pos[0]) * a
        y = (pos1[0] - self.green_pos[1]) * b
        z = (((pos1)[1] + (pos2)[1])) * (a + b) / 2
        z_1 = -pos1[1] + self.green_pos[2]
        z_2 = -pos2[1] + self.green_pos[2]
        if ((z - self.yellow_pos[2]) < 0):
            z = 103

        z = (b * z_1 + a * z_2) / 2
        return np.array([x, y, z])

    # get statistics of end-effector
    def detect_end_effector(self):
        red1 = self.detect_red(self.cv_image1)
        red2 = self.detect_red(self.cv_image2)
        red = self.calc_pos(red1, red2)
        return red

        # Calculate the forward kinematics

    def forward_kinematics(self, j1, j2, j3):
        end_effector = np.array([(-16 * math.cos(j1 + j2) + 16 * math.cos(j1 - j2) - 7 * math.cos(
            j1 + j2 + j3) + 7 * math.cos(j1 - j2 + j3) - 7 * math.cos(j1 + j2 - j3) + 7 * math.cos(
            j1 - j2 - j3) + 14 * math.sin(j1 + j3) - 14 * math.sin(j1 - j3)) / 10,
                                 (-14 * math.cos(j1 + j3) + 14 * math.cos(j1 - j3) - 16 * math.sin(
                                     j1 + j2) + 16 * math.sin(j1 - j2) - 7 * math.sin(j1 + j2 + j3) + 7 * math.sin(
                                     j1 - j2 + j3) - 7 * math.sin(j1 + j2 - j3) + 7 * math.sin(j1 - j2 - j3)) / 10,
                                 (16 * math.cos(j2) + 7 * math.cos(j2 + j3) + 7 * math.cos(j2 - j3) + 20) / 5])
        return end_effector

    # Calculate the robot Jacobian
    def calculate_jacobian(self, j1, j2, j3):
        jacobian = np.array([[(7 * sin(j1 + j3 + j2) - 7 * sin(j1 + j3 - j2) + 14 * cos(j1 + j3) + 7 * sin(
            j1 - j3 + j2) - 7 * sin(j1 - j3 - j2) - 14 * cos(j1 - j3) + 16 * sin(j1 + j2) - 16 * sin(j1 - j2)) / 10, (
                                          7 * sin(j2 + j3 + j1) - 7 * sin(j2 + j3 - j1) + 7 * sin(
                                      j2 - j3 + j1) - 7 * sin(j2 - j3 - j1) + 16 * sin(j2 + j1) - 16 * sin(
                                      j2 - j1)) / 10, (7 * (
                    sin(j3 + j2 + j1) - sin(j3 + j2 - j1) - sin(j3 - j2 + j1) + sin(j3 - j2 - j1) + 2 * cos(
                j3 + j1) + 2 * cos(j3 - j1))) / 10],
                             [(-7 * cos(j1 + j3 + j2) + 7 * cos(j1 + j3 - j2) + 14 * sin(j1 + j3) - 7 * cos(
                                 j1 - j3 + j2) + 7 * cos(j1 - j3 - j2) - 14 * sin(j1 - j3) - 16 * cos(
                                 j1 + j2) + 16 * cos(j1 - j2)) / 10, -(
                                         7 * cos(j2 + j3 + j1) + 7 * cos(j2 + j3 - j1) + 7 * cos(
                                     j2 - j3 + j1) + 7 * cos(j2 - j3 - j1) + 16 * cos(j2 + j1) + 16 * cos(
                                     j2 - j1)) / 10, -(7 * (
                                         cos(j3 + j2 + j1) + cos(j3 + j2 - j1) - cos(j3 - j2 + j1) - cos(
                                     j3 - j2 - j1) - 2 * sin(j3 + j1) + 2 * sin(j3 - j1))) / 10],
                             [0, -(7 * sin(j2 + j3) + 7 * sin(j2 - j3) + 16 * sin(j2)) / 5,
                              -(7 * (sin(j3 + j2) + sin(j3 - j2))) / 5]])
        return jacobian

    # Estimate control inputs for open-loop control
    def control_open(self, j1, j3, j4):
        # estimate time step
        cur_time = rospy.get_time()
        dt = cur_time - self.time_previous_step2
        self.time_previous_step2 = cur_time
        q = [self.joint1, self.joint3, self.joint4]  # estimate initial value of joints'
        J_inv = np.linalg.pinv(self.calculate_jacobian(j1, j3, j4))  # calculating the psudeo inverse of Jacobian
        pos = self.end_effector.data
        # desired trajectory
        pos_d = self.target.data
        # estimate derivative of desired trajectory
        self.error = (pos_d - pos) / dt
        q_d = q + (dt * np.dot(J_inv, self.error.transpose()))  # desired joint angles to follow the trajectory
        return q_d

    def callback_image1(self, data):
        # Recieve the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def callback_image2(self, data):
        # Recieve the image
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        x_e_image = self.detect_end_effector()
        self.end_effector = Float64MultiArray()
        self.end_effector.data = x_e_image

        # send control commands to joints
        self.x = Float64()
        self.x.data = x_e_image[0]
        self.y = Float64()
        self.y.data = x_e_image[1]
        self.z = Float64()
        self.z.data = x_e_image[2]
        self.x_pub.publish(self.x)
        self.y_pub.publish(self.y)
        self.z_pub.publish(self.z)
        self.end_effector_pub.publish(self.end_effector)

    def callback_target(self, data):

        self.target = data
        target = data.data
        self.xt_pub.publish(target[0])
        self.yt_pub.publish(target[1])
        self.zt_pub.publish(target[2])

    # Recieve data, process it
    def callback_angle1(self, data):
        self.joint1 = data.data

    # Recieve data, process it
    def callback_angle3(self, data):
        self.joint3 = data.data

    # Recieve data, process it
    def callback_angle4(self, data):
        self.joint4 = data.data
        moves = self.control_open(self.joint1, self.joint3, self.joint4)
        self.robot_joint1_pub.publish(moves[0])
        self.robot_joint3_pub.publish(moves[1])
        self.robot_joint4_pub.publish(moves[2])


# call the class
def main(args):
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
