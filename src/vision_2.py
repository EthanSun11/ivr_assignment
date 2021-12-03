#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import math
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64, Int16MultiArray
from cv_bridge import CvBridge, CvBridgeError

last_blue_image1 = [0, 0]
last_red_image1 = [0, 0]
last_blue_image2 = [0, 0]
last_red_image2 = [0, 0]


class image_converter:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)
        # initialize a publisher to send images from camera1 to a topic named image_topic1
        self.image_pub1 = rospy.Publisher("image_topic1", Image, queue_size=1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback1)
        # initialize a publisher to send images from camera2 to a topic named image_topic2
        self.image_pub2 = rospy.Publisher("image_topic2", Image, queue_size=1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()
        # initialize a publisher to send joints' angular position to a topic called joints_pos
        self.joints_pub = rospy.Publisher("joints_pos", Float64MultiArray, queue_size=10)
        # record the begining time
        self.time_trajectory = rospy.get_time()
        # self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
        self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')

        self.joint1_pub = rospy.Publisher("joint_angle_1", Float64, queue_size=10)
        self.joint3_pub = rospy.Publisher("joint_angle_3", Float64, queue_size=10)
        self.joint4_pub = rospy.Publisher("joint_angle_4", Float64, queue_size=10)

        # hardcoded them so that i do not calculate ratio every time
        self.pixel2meter_image1 = 0.034777349758716214
        self.pixel2meter_image2 = 0.03598350534333363

        # hardcode green and yellow coordinates since yellow and green spheres do not move and this improves accuracy
        self.green_pos = np.array([399, 400, 543])
        self.yellow_pos = np.array([399, 399, 440])

        # publish angle
        self.det1_pub = rospy.Publisher("det1", Float64, queue_size=10)
        self.det3_pub = rospy.Publisher("det3", Float64, queue_size=10)
        self.det4_pub = rospy.Publisher("det4", Float64, queue_size=10)
        rospy.sleep(0.4)

    def detect_red(self, image):
        # Isolate the red colour in the image as a binary image
        mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
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

    # Detecting the centre of the blue circle
    def detect_blue(self, image):
        mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        # Calculate pixel coordinates for the centre of the blob
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            if self.cv_image1 is image:
                last_blue_image1[0] = cx
                last_blue_image1[1] = cy
                return np.array([cx, cy])
            else:
                last_blue_image2[0] = cx
                last_blue_image2[1] = cy
                return np.array([cx, cy])
        # this is in case blue is blocked by yellow
        else:
            if self.cv_image1 is image:
                return np.array(last_blue_image1)
            else:
                return np.array(last_blue_image2)

    # Detecting the centre of the yellow circle
    def detect_yellow(self, image):
        mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])

    # Detecting the centre of the green circle
    def detect_green(self, image):
        mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])

    def control_move(self):
        # estimate time step
        cur_time = np.array([rospy.get_time() - self.time_trajectory])
        joint1 = (math.pi) * math.sin(math.pi * cur_time / 28)
        joint3 = (math.pi / 2) * math.sin(math.pi * cur_time / 20)
        joint4 = (math.pi / 2) * math.sin(math.pi * cur_time / 18)
        return [joint1, joint3, joint4]

    def vector2angle(self, vect1, vect2):
        dot_vect1_vect2 = np.dot(vect1, vect2)
        length_vect1 = np.linalg.norm(vect1)
        length_vect2 = np.linalg.norm(vect2)
        return np.arccos(dot_vect1_vect2 / (length_vect1 * length_vect2))

    def calc_pos(self, pos1, pos2):
        x = pos2[0] - self.green_pos[0]
        y = pos1[0] - self.green_pos[1]
        if ((-pos1[1] + self.yellow_pos[2]) < 0 or (-pos2[1] + self.yellow_pos[2]) < 0):
            return np.array([x, y, 103])
        z_1 = -pos1[1] + self.green_pos[2]
        z_2 = -pos2[1] + self.green_pos[2]
        z = (z_1 + z_2) / 2
        return np.array([x, y, z])

    # calculate angles in vectors
    def calc_angles(self):
        # a = self.pixel2meter_image1
        # b = self.pixel2meter_image2
        green = np.array([0, 0, 0])
        yellow = np.array([0, 0, 103])
        blue1 = (self.detect_blue(self.cv_image1))
        blue2 = (self.detect_blue(self.cv_image2))
        blue_image = np.array([blue2[0], blue1[0], (blue1[1] + blue2[1]) / 2])
        blue = self.calc_pos(blue1, blue2)
        red1 = (self.detect_red(self.cv_image1))
        red2 = (self.detect_red(self.cv_image2))
        red_image = np.array([red2[0], red1[0], (red1[1] + red2[1]) / 2])
        red = self.calc_pos(red1, red2)

        blue2red = red - blue
        yellow2blue = blue - yellow
        # joint3 is the angle between the z-axis and the link from yellow to blue.
        j3 = self.vector2angle(yellow2blue, yellow)
        if j3 > (np.pi) / 2:
            j3 = np.pi - j3
        elif j3 < -(np.pi) / 2:
            j3 = -np.pi + j3

        # we can observe the rotation of joint1 by comparing
        # the projection of link from yellow to blue and the direction of original y-zxis.
        j1 = self.vector2angle(np.array([yellow2blue[0], yellow2blue[1]]), np.array([0, -1]))
        if yellow2blue[0] < 0:
            j1 = -j1

        # calculate rotation around joint 4 by finding angle between links.
        j4 = abs(self.vector2angle(yellow2blue, blue2red))
        if j4 > (np.pi) / 2:
            j4 = np.pi - j4
        elif j4 < -(np.pi) / 2:
            j4 = -np.pi + j4
        new_x = np.array([np.cos(j1), np.sin(j1), 0])
        if np.dot(blue2red, new_x) < 0:
            j4 = -j4

        return [j1, j3, j4]

    def callback1(self, data):
        # Recieve the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        # Uncomment if you want to save the image
        # cv2.imwrite('image_copy.png', cv_image)

        # Publish the results
        try:
            self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
        except CvBridgeError as e:
            print(e)

    # Recieve data, process it, and publish
    def callback2(self, data):
        # Recieve the image
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # The published joints detected by the vision are placed in array
        self.joints = Float64MultiArray()
        self.joints.data = self.calc_angles()
        self.joints_pub.publish(self.joints)

        j1 = self.joints.data[0]
        j3 = self.joints.data[1]
        j4 = self.joints.data[2]
        self.joint1_pub.publish(j1)
        self.joint3_pub.publish(j3)
        self.joint4_pub.publish(j4)
        self.det1_pub.publish(j1)
        self.det3_pub.publish(j3)
        self.det4_pub.publish(j4)


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
