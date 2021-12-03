#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send messages to a topic named image_topic
    # self.image_pub = rospy.Publisher("image_topic",Image, queue_size = 1)
    # # initialize a publisher to send joints' angular position to a topic called joints_pos
    # self.joints_pub = rospy.Publisher("joints_pos",Float64MultiArray, queue_size=10)
    # initialize a publisher to send robot end-effector position
    self.end_effector_pub = rospy.Publisher("end_effector_prediction",Float64MultiArray, queue_size=10)
    # initialize a publisher to send desired trajectory
    # self.trajectory_pub = rospy.Publisher("trajectory",Float64MultiArray, queue_size=10)
    # initialize a publisher to send joints' angular position to the robot
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub = rospy.Subscriber("/robot/camera1/image_raw",Image,self.callback_image1)
    self.image_sub2 = rospy.Subscriber("/robot/camera2/image_raw",Image,self.callback_image2)
    # record the begining time
    self.time_trajectory = rospy.get_time()
    # initialize errors
    # self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
    self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')
    # initialize error and derivative of error for trajectory tracking
    self.error = np.array([0.0,0.0], dtype='float64')
    self.error_d = np.array([0.0,0.0], dtype='float64')

    # subscribe to detected angles
    self.image_sub = rospy.Subscriber("joints_pos", Float64MultiArray, self.callback_detected)
    # publish predicted end_effector
    self.fk_pub = rospy.Publisher("fk",Float64MultiArray, queue_size=10)
    # subscribe to the target position
    self.target_sub = rospy.Subscriber("target_pos",Float64MultiArray,self.callback_target)


    # hardcoded them so that i do not calculate ratio every time
    self.pixel2meter_image1 = 0.034777349758716214
    self.pixel2meter_image2 = 0.03598350534333363
    self.yellow_pos = np.array([399, 399, 440])




  def detect_red(self, image):
      # Isolate the blue colour in the image as a binary image
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


  # Detecting the centre of the green circle
  # def detect_green(self,image):
  #     mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
  #     kernel = np.ones((5, 5), np.uint8)
  #     mask = cv2.dilate(mask, kernel, iterations=3)
  #     M = cv2.moments(mask)
  #     cx = int(M['m10'] / M['m00'])
  #     cy = int(M['m01'] / M['m00'])
  #     return np.array([cx, cy])
  #
  # # Detecting the centre of the blue circle
  # def detect_blue(self,image):
  #     mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
  #     kernel = np.ones((5, 5), np.uint8)
  #     mask = cv2.dilate(mask, kernel, iterations=3)
  #     M = cv2.moments(mask)
  #     cx = int(M['m10'] / M['m00'])
  #     cy = int(M['m01'] / M['m00'])
  #     return np.array([cx, cy])
  #
  # # Detecting the centre of the yellow circle
  # def detect_yellow(self,image):
  #     mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
  #     kernel = np.ones((5, 5), np.uint8)
  #     mask = cv2.dilate(mask, kernel, iterations=3)
  #     M = cv2.moments(mask)
  #     cx = int(M['m10'] / M['m00'])
  #     cy = int(M['m01'] / M['m00'])
  #     return np.array([cx, cy])
  #
  # # Calculate the conversion from pixel to meter
  # def pixel2meter(self,image):
  #     # Obtain the centre of each coloured blob
  #     circle1Pos = self.detect_yellow(image)
  #     circle2Pos = self.detect_green(image)
  #     # find the distance between two circles
  #     dist = np.sum((circle1Pos - circle2Pos)**2)
  #     return 4 / np.sqrt(dist)

    # Calculate the relevant joint angles from the image
  # def detect_joint_angles(self,image):
  #   a = self.pixel2meter(image)
  #   # Obtain the centre of each coloured blob
  #   center = a * self.detect_yellow(image)
  #   circle1Pos = a * self.detect_blue(image)
  #   circle2Pos = a * self.detect_green(image)
  #   circle3Pos = a * self.detect_red(image)
  #   # Solve using trigonometry
  #   ja1 = np.arctan2(center[0]- circle1Pos[0], center[1] - circle1Pos[1])
  #   ja2 = np.arctan2(circle1Pos[0]-circle2Pos[0], circle1Pos[1]-circle2Pos[1]) - ja1
  #   ja3 = np.arctan2(circle2Pos[0]-circle3Pos[0], circle2Pos[1]-circle3Pos[1]) - ja2 - ja1
  #   return np.array([ja1, ja2, ja3])

  def calc_pos(self, pos1, pos2):
        x = pos2[0] - self.green_pos[0]
        y = pos1[1] - self.green_pos[1]
        z_1 = -pos1[1] + self.green_pos[2]
        z_2 = -pos2[1] + self.green_pos[2]
        z = (z_1 + z_2) / 2
        if ((z - self.yellow_pos[2]) < 0):
            z = 103
        return np.array([x, y, z])

  def detect_end_effector(self):
      a = self.pixel2meter_image1
      b = self.pixel2meter_image2
      # green = np.array([0, 0, 0])
      # yellow = np.array([0, 0, 103])
      # blue1 = a * (self.detect_blue(self.cv_image1))
      # blue2 = b * (self.detect_blue(self.cv_image2))
      # blue = self.calc_pos(blue1, blue2)
      red1 = a * (self.detect_red(self.cv_image1))
      red2 = b * (self.detect_red(self.cv_image2))
      red = self.calc_pos(red1, red2)
      return red
    # detect robot end-effector from the image
  #
  # def detect_end_effector(self,image):
  #   a = self.pixel2meter(image)
  #   endPos = a * (self.detect_yellow(image) - self.detect_red(image))
  #   return endPos

  # Define a circular trajectory
  # def trajectory(self):
  #   # get current time
  #   cur_time = np.array([rospy.get_time() - self.time_trajectory])
  #   x_d = float(5.5* np.cos(cur_time * np.pi/100))
  #   y_d = float(5.5 + np.absolute(1.5* np.sin(cur_time * np.pi/100)))
  #   return np.array([x_d, y_d])

  # Calculate the forward kinematics
  def forward_kinematics(self,j1,j2,j3):
      end_effector = np.array([(-16*math.cos(j1 +j3)+ 16*math.cos(j1-j2) - 7 *math.cos(j1+j2+j3) + 7* math.cos(j1-j2+j3) - 7 *math.cos(j1+j2 - j3) + 7*math.cos(j1-j2-j3) +14 *math.sin(j1+j3) - 14 *math.sin(j1-j3))/10,
                               [(-14*math.cos(j1 + j3) + 14 * math.cos(j1-j3) - 16 * math.sin(j1 + j2) + 16 * sin(j1-j2) - 7 * math.sin(j1+j2+j3) + 7 * math.sin(j1-j2 + j3) - 7 *math.sin(j1-j2+j3) - 7 *math.sin(j1+j2-j3) + 7 * math.sin(j1-j2-j3))/10],
                               [(16* math.cos(j2) + 7 * math.cos(j2 + j3) + 7 * math.cos(j2-j3) +20)/5]])
      return end_effector

  # Calculate the robot Jacobian
  def calculate_jacobian(self):
    joints = self.joints
    jacobian = np.array([[3 * np.cos(joints[0]) + 3 * np.cos(joints[0]+joints[1]) + 3 *np.cos(joints.sum()), 3 * np.cos(joints[0]+joints[1]) + 3 *np.cos(joints.sum()),  3 *np.cos(joints.sum())], [-3 * np.sin(joints[0]) - 3 * np.sin(joints[0]+joints[1]) - 3 * np.sin(joints.sum()), - 3 * np.sin(joints[0]+joints[1]) - 3 * np.sin(joints.sum()), - 3 * np.sin(joints.sum())]])
    return jacobian

  # Estimate control inputs for open-loop control
  def control_open(self):
    # estimate time step
    cur_time = rospy.get_time()
    dt = cur_time - self.time_previous_step2
    self.time_previous_step2 = cur_time
    q = self.joints # estimate initial value of joints'
    J_inv = np.linalg.pinv(self.calculate_jacobian())  # calculating the psudeo inverse of Jacobian
    pos = self.pos
    # desired trajectory
    pos_d= self.target
    # estimate derivative of desired trajectory
    self.error = (pos_d - pos)/dt
    q_d = q + (dt * np.dot(J_inv, self.error.transpose()))  # desired joint angles to follow the trajectory
    return q_d

  # Recieve data, process it, and publish
  def callback_detected(self,data):
    # Recieve the image
    # try:
    #   cv_
    # except CvBridgeError as e:
    #   print(e)

    j1 = data[0]
    j3 = data[1]
    j4 = data[2]
    self.joints = data
    self.pos = self.forward_kinematics(j1,j3,j4)




  def control_move(self):
        # estimate time step
        cur_time = np.array([rospy.get_time() - self.time_trajectory])
        # self.time_previous_step2 = cur_time
        joint2 = (math.pi / 2) * math.sin(math.pi * cur_time / 15)
        joint3 = (math.pi / 2) * math.sin(math.pi * cur_time / 20)
        joint4 = (math.pi / 2) * math.sin(math.pi * cur_time / 18)
        return [joint2, joint3, joint4]
    # Perform image processing task (your code goes here)
    # The image is loaded as cv_imag

    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)


    # compare the estimated position of robot end-effector calculated from images with forward kinematics(lab 3)
    #
    # # Publishing the desired trajectory on a topic named trajectory(lab 3)
    # x_d = self.trajectory()    # getting the desired trajectory
    # self.trajectory_desired= Float64MultiArray()
    # self.trajectory_desired.data=x_d



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

      # send control commands to joints (lab 3)
      q_d = self.control_open()
      self.joint1 = Float64()
      self.joint1.data = q_d[0]
      self.joint2 = Float64()
      self.joint2.data = q_d[1]
      self.joint3 = Float64()
      self.joint3.data = q_d[2]
      try:
          self.fk_pub.publish(self.pos)
          self.end_effector_pub.publish(self.end_effector)
      # self.trajectory_pub.publish(self.trajectory_desired)
          self.robot_joint1_pub.publish(self.joint1)
          self.robot_joint2_pub.publish(self.joint2)
          self.robot_joint3_pub.publish(self.joint3)
      except CvBridgeError as e:
          print(e)

  def callback_target(self,data):
       self.target = data

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
