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

class vision_1:
	
  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    #initialize a publisher with each joint
    self.joint2_swine = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size = 3)
    self.joint3_swine = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size = 3)
    self.joint4_swine = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size = 3)
    self.joint_angles_est = rospy.Publisher("/robot/joint_estimated", Float64MultiArray, queue_size = 3)
    
  # In this method you can focus on detecting the centre of the red circle
  def detect_red(self,image):
      # Isolate the blue colour in the image as a binary image
      mask = cv2.inRange(image1, (0, 0, 100), (0, 0, 255))
      # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      # Obtain the moments of the binary image
      M = cv2.moments(mask)
      # Calculate pixel coordinates for the centre of the blob
      cz = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cz, cy])
 

  # Detecting the centre of the green circle
  def detect_green(self,image):
      mask = cv2.inRange(image1, (0, 100, 0), (0, 255, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cz = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cz, cy])


  # Detecting the centre of the blue circle
  def detect_blue(self,image):
      mask = cv2.inRange(image1, (100, 0, 0), (255, 0, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cz = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cz, cy])

  # Detecting the centre of the yellow circle
  def detect_yellow(self,image):
      mask = cv2.inRange(image1, (0, 100, 100), (0, 255, 255))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cz = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cz, cy])


  # Calculate the conversion from pixel to meter
  def pixel2meter(self,image):
      # Obtain the centre of each coloured blob
      circle1Pos = self.detect_yellow(image1)
      circle2Pos = self.detect_green(image1)
      # find the distance between two circles
      dist = np.sum((circle1Pos - circle2Pos)**2)
      return 4 / np.sqrt(dist)
    
  def change_from_planar_to_spatial(self,image1,image2):
    a1 = self.pixel2meter(image1)
    a2 = self.pixel2meter(image2)
    blob_red1 = a1 * self.detect_red(image1)
    blob_green1 = a1 * self.detect_green(image1)
    blob_yellow1 = a1 * self.detect_yellow(image1)
    blob_blue1 = a1 * self.detect_blue(image1)
    blob_red2 = a2 * self.detect_red(image2)
    blob_green2 = a2 * self.detect_green(image2)
    blob_yellow2 = a2 * self.detect_yellow(image2)
    blob_blue2 = a2 * self.detect_blue(image2)
    blob_red_2d3 = np.array([blob_red2[0], blob_red1[1], -(blob_red1[0]+blob_red2[1])/2])
    blob_green_2d3 = np.array([blob_green2[0], blob_green1[1], -(blob_green1[0]+blob_green2[1])/2])
    blob_yellow_2d3 = np.array([blob_yellow2[0], blob_yellow1[1], -(blob_yellow1[0]+blob_yellow2[1])/2])
    blob_blue_2d3 = np.array([blob_blue2[0], blob_blue1[1], -(blob_blue1[0]+blob_blue2[1])/2])
    return np.array([blob_red_2d3, blob_green_2d3, blob_yellow_2d3, blob_blue_2d3])
    
  # Calculate the relevant joint angles from the image
  def detect_joint_angles(self, image1, image2):
    red_3d = self.change_from_planar_to_spatial(image1, image2)[0]
    green_3d = self.change_from_planar_to_spatial(image1, image2)[1]
    blue_3d = self.change_from_planar_to_spatial(image1, image2)[2]
    yellow_3d = self.change_from_planar_to_spatial(image1, image2)[3]
    joint_pos = [0.0, 0.0, 0.0]
    #joint angle 2(yellow), around y-axis
    joint_pos[0] = np.arctan2(yellow_3d[0]-blue_3d[0], yellow_3d[2]-blue_3d[2])
    #joint angle 3(yellow), around x-axis
    joint_pos[1] = np.arctan2(yellow_3d[2]-blue_3d[2], yellow_3d[1]-blue_3d[1]) - joint_pos[0]
    #joint angle 4(blue), around y-axis
    joint_pos[2] = np.arctan2(blue_3d[0]-red_3d[0], blue_3d[2]-red_3d[2]) - joint_pos[1] - joint_pos[0]
    return joint_pos
    
  
  #message.data = detect_joint_angles(self, image1, image2)
      
  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
     
      
  def callback2(self,data):

    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)
    im1=cv2.imshow('window1', self.cv_image1)
    cv2.waitKey(1)
    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
    except CvBridgeError as e:
      print(e)
      
    
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)
    im2=cv2.imshow('window2', self.cv_image2)
    cv2.waitKey(1)

    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
    except CvBridgeError as e:
      print(e)

      
    def update_joint2(self, t):
      sinu_node = np.pi/2.0 * np.sin(np.pi/15.0 * t)
      self.joint2_swine.publish(sinu_node)
      
    def update_joint3(self, t):
      sinu_node = np.pi/2.0 * np.sin(np.pi/20.0 * t)
      self.joint3_swine.publish(sinu_node)
      
    def update_joint4(self, t):
      sinu_node = np.pi/2.0 * np.sin(np.pi/18.0 * t)
      self.joint4_swine.publish(sinu_node)
      
    message = Float64MultiArray(data = self.detect_joint_angles)
    self.joint_angles_est.publish(message)

# call the class
def main(args):
  ic = vision_1()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
