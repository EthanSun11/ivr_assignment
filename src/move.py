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

class image_converter:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('move', anonymous=True)
      
        # initialize a publisher to send joints' angular position to the robot
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        # record the begining time
        self.time_trajectory = rospy.get_time()
        #publish angle
        self.real1_pub = rospy.Publisher("real1", Float64, queue_size=10)
        self.real3_pub = rospy.Publisher("real3", Float64, queue_size=10)
        self.real4_pub = rospy.Publisher("real4", Float64, queue_size=10)
        self.real_pub = rospy.Publisher("real", Float64MultiArray, queue_size=10)
        # run callback
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback)
       
        rospy.sleep(0.4)

    def control_move(self):
        # estimate time step
        cur_time = np.array([rospy.get_time() - self.time_trajectory])
        joint1 = (math.pi) * math.sin(math.pi * cur_time / 28)
        joint3 = (math.pi / 2) * math.sin(math.pi * cur_time / 20)
        joint4 = (math.pi / 2) * math.sin(math.pi * cur_time / 18)
        return [joint1, joint3, joint4]

    def callback(self,data):
        angles = self.control_move()
        self.joint1 = Float64()
        self.joint1.data = angles[0]
        self.joint3 = Float64()
        self.joint3.data = angles[1]
        self.joint4 = Float64()
        self.joint4.data = angles[2]
        real = Float64MultiArray()
        real.data = np.array([self.joint1.data,self.joint3.data,self.joint4.data])
        # Publish the results
        try:
            self.real_pub.publish(real)
            self.real1_pub.publish(self.joint1.data)
            self.real3_pub.publish(self.joint3.data)
            self.real4_pub.publish(self.joint4.data)
            self.robot_joint1_pub.publish(self.joint1)
            self.robot_joint3_pub.publish(self.joint3)
            self.robot_joint4_pub.publish(self.joint4)
    
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
