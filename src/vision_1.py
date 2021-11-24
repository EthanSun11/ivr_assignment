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

last_blue_image1 =[0,0]
last_red_image1  =[0,0]
last_blue_image2 =[0,0]
last_red_image2  =[0,0]


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

        # initialize a publisher to send joints' angular position to the robot
        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        # record the begining time
        self.time_trajectory = rospy.get_time()
        # initialize errors
        # self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
        self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')

        # initialize a publisher for end effector position
        self.end_eff_pub = rospy.Publisher("/end_pos", Float64MultiArray, queue_size=10)

        self.joint2_pub = rospy.Publisher("/joints_ang2", Float64, queue_size=10)
        self.joint3_pub = rospy.Publisher("/joints_ang3", Float64, queue_size=10)
        self.joint4_pub = rospy.Publisher("/joints_ang4", Float64, queue_size=10)

        # hardcoded them so that i do not calculate ratio every time
        self.pixel2meter_image1 = 0.034777349758716214
        self.pixel2meter_image2 = 0.03598350534333363

        # hardcode green and yellow coordinates since yellow and green spheres do not move and this improves accuracy
        self.green_pos = np.array([399, 400, 543])
        self.yellow_pos = np.array([399, 399, 440])

        rospy.sleep(0.4)
        

    def detect_red(self, image):
        # Isolate the blue colour in the image as a binary image
        mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
        # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        # Obtain the moments of the binary image
        M = cv2.moments(mask)
        if M['m00'] != 0 :
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
            #this is in case red is blocked by blue
        else:
            if image is self.cv_image1:
                return np.array(last_red_image1)
            else :
                return np.array(last_red_image2)

    # Detecting the centre of the blue circle
    def detect_blue(self, image):
        mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        # Calculate pixel coordinates for the centre of the blob
        if M['m00'] != 0 :
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
        #this is in case red is blocked by blue
        else:
            if self.cv_image1 is image:
                    return np.array(last_blue_image1)
            else :
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
        # self.time_previous_step2 = cur_time
        joint2 = (math.pi / 2) * math.sin(math.pi * cur_time / 15)
        joint3 = (math.pi / 2) * math.sin(math.pi * cur_time / 20)
        joint4 = (math.pi / 2) * math.sin(math.pi * cur_time / 18)
        return [joint2, joint3, joint4]

    def length_of_vector(self, vect):
        squared_sum = np.sum(vect ** 2)
        return np.sqrt(squared_sum)

    def vector2angle(self, vect1, vect2):
        dot_vect1_vect2 = np.dot(vect1, vect2)
        length_vect1 = self.length_of_vector(vect1)
        length_vect2 = self.length_of_vector(vect2)
        return np.arccos(dot_vect1_vect2 / (length_vect1 * length_vect2))

    def calc_pos(self, pos1, pos2):
        x = pos2[0] - self.green_pos[0]
        y = pos1[1] - self.green_pos[1]
        z_1 = -pos1[1] + self.green_pos[2]
        z_2 = -pos2[1] + self.green_pos[2]
        z = (z_1 + z_2) / 2
        if ((z - self.yellow_pos[2]) < 0):
            z = 103
        return np.array([x, y, z])

    # calculate angles in naive way
    def calc_angles(self):
        a = self.pixel2meter_image1
        b = self.pixel2meter_image2
        green = np.array([0, 0, 0])
        yellow = np.array([0, 0, 103])
        blue1 = a * (self.detect_blue(self.cv_image1))
        blue2 = b * (self.detect_blue(self.cv_image2))
        blue = self.calc_pos(blue1, blue2)
        red1 = a * (self.detect_red(self.cv_image1))
        red2 = b * (self.detect_red(self.cv_image2))
        red = self.calc_pos(red1, red2)

        blue2red = red - blue
        yellow2blue = blue - yellow

        
        # calculate rotation around y-axis using the new x-axis and origin x-axis.
        new_x = np.cross(np.array([0, 1, 0]), yellow2blue)
        j2 = self.vector2angle(new_x, np.array([1, 0, 0]))
        if j2 > (np.pi) / 2:
            j2 = np.pi - j2
        elif j2 < -(np.pi) / 2:
            j2 = - np.pi + j2
        
        
        # calculate rotation around x-axis by cross product origin y-axis
        j3 = self.vector2angle(yellow2blue, np.array([0, 1, 0])) # - np.pi/2
        if j3 > (np.pi) / 2:
            j3 = np.pi - j3
        elif j3 < -(np.pi) / 2:
            j3 = -np.pi + j3
        

        # calculate rotation around joint 4 by finding angle between vectors.
        j4 = self.vector2angle(yellow2blue, blue2red)
        if j4 > (np.pi) / 2:
            j4 = np.pi - j4
        elif j4 < -(np.pi) / 2:
            j4 = -np.pi + j4
        

        return [j2, j3, j4]

    # Recieve data, process it, and publish
    def callback2(self, data):
        # Recieve the image
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        # Uncomment if you want to save the image
        # cv2.imwrite('image_copy.png', cv_image)
        # im2=cv2.imshow('window2', self.cv_image2)
        # cv2.waitKey(1)

        # image = np.concatenate((self.cv_image1, self.cv_image2), axis=1)
        # im=cv2.imshow('camera1 and camera2', image)
        # cv2.waitKey(1)

        angles = self.control_move()
        self.joint2 = Float64()
        self.joint2.data = angles[0]
        self.joint3 = Float64()
        self.joint3.data = angles[1]
        self.joint4 = Float64()
        self.joint4.data = angles[2]
        # Publish the results
        try:

            # The published joints detected by the vision are placed in array
            self.joints = Float64MultiArray()
            self.joints.data = self.calc_angles()
            self.joints_pub.publish(self.joints)

            j2 = self.joints.data[0]
            j3 = self.joints.data[1]
            j4 = self.joints.data[2]
            self.joint2_pub.publish(j2)
            self.joint3_pub.publish(j3)
            self.joint4_pub.publish(j4)
            # print([math.degrees(self.joints.data[0]),math.degrees(self.joints.data[1]),math.degrees(self.joints.data[2] )])
            print('differences')
            print([math.degrees(j2) - math.degrees(self.joint2.data),
                   math.degrees(j3) - math.degrees(self.joint3.data),
                   math.degrees(j4) - math.degrees(self.joint4.data)])

            self.robot_joint2_pub.publish(self.joint2)
            self.robot_joint3_pub.publish(self.joint3)
            self.robot_joint4_pub.publish(self.joint4)
    
        except CvBridgeError as e:
            print(e)

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
