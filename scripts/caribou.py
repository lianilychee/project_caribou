#!/usr/bin/env python

"""
Neato control program to make a robot follow a line (like a roadway) and react 
to signs in its path.
"""

import rospy
from geometry_msgs.msg import Twist, PoseWithCovariance, Pose, Point, Vector3
from sensor_msgs.msg import LaserScan, Image
import math
import numpy as np
import cv2
from cv_bridge import CvBridge
import helper_functions as hp

rotate_speed_limit = 0.3
DRIVE = 0
STOP = 1
LOOK_BOTH_WAYS = 2


class Controller:
  def __init__(self):
    rospy.init_node('caribou')

    self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    self.command = Twist()

    self.threshold = 0 # TODO: CHANGE THIS NUMBER
    self.bridge = CvBridge()

    ##### WINDOW SIZE #####
    self.win_size = (640,480)
    self.win_height_cropped = 480*0.9

    ##### INITIAL COLOR THRESHOLDS #####
    self.grey_lower = 0
    self.grey_upper = 255

    ##### STATE VARIABLES #####
    self.state = DRIVE
    self.pause_duration = rospy.Duration(3)
    self.ignore_stop_sign_threshold = self.pause_duration + rospy.Duration(3)
    self.last_stop_sign = rospy.Time.now() - self.ignore_stop_sign_threshold

    rospy.Subscriber('/camera/image_raw', Image, self.react_to_image)

    cv2.setMouseCallback('video_window', self.process_mouse_event)
    cv2.namedWindow('set_bounds')
    cv2.namedWindow('bw_window_cropped')
    cv2.namedWindow('Output')

    ##### INITIALIZE SIFT #####
    self.sift = cv2.SIFT()
    self.bf = cv2.BFMatcher()
    self.past_descriptors = []
   
    ##### SLIDERS #####

    cv2.createTrackbar('grey l', 'set_bounds', 0, 255,
        self.set_grey_lower)

    cv2.createTrackbar('grey u', 'set_bounds', 0, 255,
        self.set_grey_upper)

    self.b_l = 128
    cv2.createTrackbar('B l', 'set_bounds', 0,255, 
      self.set_b_l)

    self.b_u = 255
    cv2.createTrackbar('B u', 'set_bounds', 0,255, 
      self.set_b_u)

    self.g_l = 128
    cv2.createTrackbar('G l', 'set_bounds', 0,255, 
      self.set_g_l)

    self.g_u = 255
    cv2.createTrackbar('G u', 'set_bounds', 0, 255,
        self.set_g_u)

    self.r_l = 128
    cv2.createTrackbar('R l', 'set_bounds', 0, 255,
        self.set_r_l)

    self.r_u = 255
    cv2.createTrackbar('R u', 'set_bounds', 0, 255,
        self.set_r_u)

    ##### COLOR PARAMETERS (hand-tweaked) #####

    self.red_lb = (70,70,250)
    self.red_ub = (90,90,255)

    self.grey_lb = (230, 230, 230)
    self.grey_ub = (255, 255, 255)

    self.stop()
    self.send()


  def process_mouse_event(self, event, x,y,flags,param):
    """ Process mouse events so that you can see the color values associated
        with a particular pixel in the camera images """

    image_info_window = 255*np.ones((500,500,3))
    cv2.putText(image_info_window,
        ('Color (b=%d,g=%d,r=%d)' %
        (self.cv_image[ty,x,0],
         self.cv_image[y,x,1],
         self.cv_image[y,x,2])),
        (5,50),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0,0,0))
    cv2.imshow('image_info', image_info_window)
    cv2.waitKey(5)

  def set_grey_lower(self, val):
    """ Use sliders to set GREY lower bound. """
    self.grey_lower = val

  def set_grey_upper(self, val):
    """ Use sliders to set GREY upper bound. """
    self.grey_upper = val

  def set_b_l(self, val):
    """ Use sliders to set BLUE lower bound. """
    self.b_l = val

  def set_b_u(self, val):
    """ Use sliders to set BLUE upper bound. """
    self.b_u = val

  def set_g_l(self, val):
    """ Use sliders to set BLUE lower bound. """
    self.g_l = val

  def set_g_u(self, val):
    """ Use sliders to set GREEN upper bound. """
    self.g_u = val

  def set_r_l(self, val):
    """ Use sliders to set RED lower bound. """
    self.r_l = val

  def set_r_u(self, val):
    """ Use sliders to set RED upper bound. """
    self.r_u = val


  def react_to_image(self, msg):
    """
    Process image messages from ROS and stash them in an attribute called
    cv_image for subsequent processing

    Grabs image stream from camera, called cv_image, and processes the image for
    line following and sign detection
    """
    self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    cv2.waitKey(5)
    if self.state == DRIVE:
      print 'react'
      direction = hp.find_line(self.cv_image, 
        (0, self.win_height_cropped), self.win_size,
        self.grey_lb, #(self.grey_lower, self.grey_lower, self.grey_lower), 
        self.grey_ub, #(self.grey_upper, self.grey_upper, self.grey_upper), 
        self.threshold)
      self.drive(direction)
      sign_test = hp.find_stop_sign(self.cv_image, self.red_lb, self.red_ub) #(self.b_l,self.g_l,self.r_l), (self.b_u,self.g_u,self.r_u))
      print sign_test
      if (sign_test and 
          (rospy.Time.now() + self.ignore_stop_sign_threshold) > 
          self.last_stop_sign):
        rospy.Timer(self.pause_duration,
            self.look_both_ways, oneshot=True)
        self.state = STOP

    elif self.state == STOP:
      print 'stop'
      self.stop()

    elif self.state == LOOK_BOTH_WAYS:
      print 'look'
      gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
      kp, des = self.sift.detectAndCompute(gray, None)
      if len(self.past_descriptors) > 10:
        previous_des = self.past_descriptors.pop(0)
        matches = self.bf.knnMatch(des, previous_des, k=2)
        # Apply ratio test
        good_count = 0
        for m,n in matches:
          if m.distance < 0.75*n.distance:
            good_count += 1
        print(str(good_count) + "/" + str(len(previous_des)))
        if good_count > 0.9*len(previous_des):
          self.state = DRIVE
      self.past_descriptors.append(des)


  def look_both_ways(self, event):
    """ Callback function to set the robot's state to LOOK_BOTH_WAYS """
    self.state = LOOK_BOTH_WAYS

  def drive(self, direction):
    """ Changes self.command in response to the direction inputed """
    if direction[1]:
      if direction[0] == 0:
        self.command.angular.z = 0
        self.command.linear.x = .1
      else:
        proportion = (float(direction[0]) / (640/2))
        self.command.angular.z = (min(proportion, rotate_speed_limit)
            if proportion > 0 else max(proportion, -rotate_speed_limit))
        self.command.linear.x = .1 * (1 - abs(proportion))
    else:
      self.stop()
    # print 'direction: ' , ((self.command.linear.x, self.command.angular.z)) #UNCOMMENT

  def stop(self):
    """ Sets self.command to stop all bot motion """
    self.command.linear.x = 0
    self.command.angular.z = 0

  def send(self):
    """ Publishes self.command to ROS """
    self.pub.publish(self.command)

controller = Controller()

while not rospy.is_shutdown():
  controller.send()
