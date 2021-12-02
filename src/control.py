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

class control:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize subscribers with the joint values
    self.joint_angle_sub = rospy.Subscriber("/robot/joint_estimated", Float64MultiArray, self.callback)
    # publishing end-effector position
    self.end_effector = rospy.Publisher("/robot/end_effector", Float64MultiArray, queue_size=3)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    
  
  def transformation_matrix(self):
  
    joint_angle1 = (self.angles[0]/180.0) * np.pi
    joint_angle3 = (self.angles[1]/180.0) * np.pi
    joint_angle4 = (self.angles[2]/180.0) * np.pi
  
    parameter_table = [[joint_angle1+np.pi/2.0, np.pi/2.0, 0, 4],
  		      [joint_angle3+np.pi/2.0, np.pi/2.0, 3.2, 0],
  		      [          joint_angle4,       0.0, 2.8, 0]]
  		     
  		     
    T1_4 = [[np.sin(joint_angle1)*np.sin(joint_angle3)*np.cos(joint_angle4) + np.sin(joint_angle1)*np.sin(joint_angle4)*np.cos(joint_angle3), 
  -np.sin(joint_angle1)*np.sin(joint_angle3)*np.sin(joint_angle4) + np.sin(joint_angle1)*np.cos(joint_angle4)*np.cos(joint_angle3), np.cos(joint_angle1), 3.2*np.sin(joint_angle1)+ 2.8*np.sin(joint_angle1)*np.sin(joint_angle3)],
  [np.sin(joint_angle3)*np.sin(joint_angle4) - np.cos(joint_angle3)*np.cos(joint_angle3), np.sin(joint_angle3)*np.cos(joint_angle4) + np.sin(joint_angle4)*np.cos(joint_angle3), 0.0, -2.8*np.cos(joint_angle3)],
  [-np.sin(joint_angle3)*np.cos(joint_angle1)*np.cos(joint_angle4) - np.sin(joint_angle4)*np.cos(joint_angle1)*np.cos(joint_angle3), 
  np.sin(joint_angle3)*np.sin(joint_angle4)*np.cos(joint_angle1) - np.cos(joint_angle1)*np.cos(joint_angle4)*np.cos(joint_angle3), np.sin(joint_angle1), -2.8*np.sin(joint_angle3)*np.cos(joint_angle1)+ 4],
  [0.0, 0.0, 0.0, 1.0]]
  
    K_q = [[3.2*np.sin(joint_angle1)+ 2.8*np.sin(joint_angle1)*np.sin(joint_angle3)],
           [-2.8*np.cos(joint_angle3)],
           [-2.8*np.sin(joint_angle3)*np.cos(joint_angle1)+ 4]]
    return K_q
  
  def callback(self, data):
    
    self.angles = data.data

    message = Float64MultiArray()
    message.data = np.array(self.transformation_matrix())
    self.end_effector.publish(message)
    print(message.data)
    
# call the class
def main(args):
  ic = control()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)

