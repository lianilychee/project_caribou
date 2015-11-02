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
import signal
import sys

##### GLOBAL SPEED CONSTANT #####
rotate_speed_limit = 0.3

##### GLOBAl STATE CONSTANTS #####
DRIVE = 0
STOP = 1
LOOK_BOTH_WAYS = 2

class Controller:
  def __init__(self):

    ##### ROS INITIALIZATION #####
    rospy.init_node('caribou')
    self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    self.command = Twist()
    self.threshold = 0 # TODO: CHANGE THIS NUMBER
    self.bridge = CvBridge()
    rospy.Subscriber('/camera/image_raw', Image, self.react_to_image)

    ##### IMAGE SIZE #####
    self.win_size = (640,480)
    self.win_height_cropped = 480*0.9

    ##### SET STATE #####
    self.state = DRIVE

    ##### INITIALIZE WINDOWS #####
    cv2.namedWindow('set_bounds')
    cv2.namedWindow('bw_window_cropped')
    cv2.namedWindow('Output')

    ##### INITIALIZE SIFT #####
    self.sift = cv2.SIFT()
    self.bf = cv2.BFMatcher()
    self.past_descriptors = []

    ##### SIGN REACTION BEHAVIOR #####
    self.pause_duration = rospy.Duration(3)
    self.ignore_stop_sign_threshold = self.pause_duration + rospy.Duration(3)
    self.last_stop_sign = rospy.Time.now() - self.ignore_stop_sign_threshold

    ##### COLOR PARAMETERS (hand-tweaked) #####
    settings_file = open('settings.txt', 'r')
    self.grey_lb = int(settings_file.readline())
    self.grey_ub = int(settings_file.readline())
    self.red_lb = eval(settings_file.readline())
    self.red_ub = eval(settings_file.readline())
    settings_file.close()
   
    ##### CALIBRATION SLIDERS #####
    cv2.createTrackbar('grey l', 'set_bounds', self.grey_lb , 255,
        self.set_grey_lower)
    cv2.createTrackbar('grey u', 'set_bounds', self.grey_ub , 255,
        self.set_grey_upper)
    cv2.createTrackbar('B l', 'set_bounds', self.red_lb[0], 255, 
      self.set_b_l)
    cv2.createTrackbar('B u', 'set_bounds', self.red_ub[0], 255, 
      self.set_b_u)
    cv2.createTrackbar('G l', 'set_bounds', self.red_lb[1] ,255, 
      self.set_g_l)
    cv2.createTrackbar('G u', 'set_bounds', self.red_ub[1], 255,
        self.set_g_u)
    cv2.createTrackbar('R l', 'set_bounds', self.red_lb[2], 255,
        self.set_r_l)
    cv2.createTrackbar('R u', 'set_bounds', self.red_ub[2], 255,
        self.set_r_u)

    ##### START OFF STOPPED #####
    self.stop()
    self.send()

  def set_grey_lower(self, val):
    """ Use sliders to set GREY lower bound. """
    self.grey_lb = val

  def set_grey_upper(self, val):
    """ Use sliders to set GREY upper bound. """
    self.grey_ub = val

  def set_b_l(self, val):
    """ Use sliders to set BLUE lower bound. """
    self.red_lb[0] = val

  def set_b_u(self, val):
    """ Use sliders to set BLUE upper bound. """
    self.red_ub[0] = val

  def set_g_l(self, val):
    """ Use sliders to set BLUE lower bound. """
    self.red_lb[1] = val

  def set_g_u(self, val):
    """ Use sliders to set GREEN upper bound. """
    self.red_ub[1] = val

  def set_r_l(self, val):
    """ Use sliders to set RED lower bound. """
    self.red_lb[2] = val

  def set_r_u(self, val):
    """ Use sliders to set RED upper bound. """
    self.red_ub[2] = val

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
      direction = hp.find_line(self.cv_image, 
        (0, self.win_height_cropped), self.win_size,
        (self.grey_lb, self.grey_lb, self.grey_lb), 
        (self.grey_ub, self.grey_ub, self.grey_ub), 
        self.threshold)
      self.drive(direction)
      sign_test = hp.find_stop_sign(self.cv_image,
          tuple(self.red_lb), tuple(self.red_ub))
      if (sign_test and 
          (rospy.Time.now() - self.ignore_stop_sign_threshold) > 
          self.last_stop_sign):
        rospy.Timer(self.pause_duration,
            self.look_both_ways, oneshot=True)
        self.state = STOP

    elif self.state == STOP:
      self.stop()

    elif self.state == LOOK_BOTH_WAYS:
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
        if good_count > 0.6*len(previous_des):
          self.state = DRIVE
      self.past_descriptors.append(des)

    cv2.imshow("Output", self.cv_image)
    cv2.waitKey(5)

  def look_both_ways(self, event):
    """ Callback function to set the robot's state to LOOK_BOTH_WAYS """
    self.last_stop_sign = rospy.Time.now()
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

  def stop(self):
    """ Sets self.command to stop all bot motion """
    self.command.linear.x = 0
    self.command.angular.z = 0

  def send(self):
    """ Publishes self.command to ROS """
    self.pub.publish(self.command)

  def signal_handler(self, signal, frame):
    """ Saves calibration settings to settings.txt file before closing """
    settings_file = open('settings.txt', 'w')
    settings_file.write(str(self.grey_lb) + '\n')
    settings_file.write(str(self.grey_ub) + '\n')
    settings_file.write(str(self.red_lb) + '\n')
    settings_file.write(str(self.red_ub) + '\n')
    settings_file.close()
    print('Exiting gracefully...')
    sys.exit(0)

controller = Controller()
signal.signal(signal.SIGINT, controller.signal_handler)

while not rospy.is_shutdown():
  controller.send()