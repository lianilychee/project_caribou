#!/usr/bin/env python

""" Have bot follow a walking person like a pet. """

import rospy
from geometry_msgs.msg import Twist, PoseWithCovariance, Pose, Point, Vector3
from sensor_msgs.msg import LaserScan, Image
import math
import numpy as np
import cv2
from cv_bridge import CvBridge
import helper_functions as hp

speed_factor = .3
rotate_speed_limit = 0.3
personal_space = .5


class Controller:
  def __init__(self):
    rospy.init_node('caribou')

    self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    self.command = Twist()

    self.threshold = 0 # TODO: CHANGE THIS NUMBER
    self.bridge = CvBridge()

    rospy.Subscriber('/camera/image_raw', Image, self.process_image)

    cv2.setMouseCallback('video_window', self.process_mouse_event)
    cv2.namedWindow('set_bounds')

   
    ##### SLIDERS #####

    self.grey_lower = 0
    cv2.createTrackbar('grey l', 'set_bounds', 0, 255,
        self.set_grey_lower)

    self.grey_upper = 255
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

    self.red_lb = (0,188,42)
    self.red_ub = (255,255,255)

    self.grey_lb = 130
    self.grey_ub = 255

    ##### WINDOW SIZE #####
    self.win_size = (640,480)
    self.win_height_cropped = 480*0.9

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
    self.grey_lower = val

  def set_grey_upper(self, val):
    self.grey_upper = val

  def set_b_l(self, val):
    self.b_l = val

  def set_b_u(self, val):
    self.b_u = val

  def set_g_l(self, val):
    self.g_l = val

  def set_g_u(self, val):
    self.g_u = val

  def set_r_l(self, val):
    self.r_l = val

  def set_r_u(self, val):
    self.r_u = val


  def process_image(self, msg):
    """
    Process image messages from ROS and stash them in an attribute called
    cv_image for subsequent processing

    Grabs image stream from camera, called cv_image, and processes the image for
    line following and sign detection
    """
    self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
    self.bw_image = cv2.inRange(self.hsv_image, self.red_lb, self.red_ub) 

    cv2.waitKey(5)

    threshold = self.threshold

    # to detect line
    direction = hp.find_line(self.cv_image, (0, self.win_height_cropped), self.win_size,(self.grey_lower,self.grey_lower,self.grey_lower), (self.grey_upper,self.grey_upper,self.grey_upper), threshold) #TODO: move these hard-coded values up into controller
    self.react(direction)


    pt1 = (100,100)
    pt2 = (300,300)


    # draw bounding box
    cv2.rectangle(self.cv_image, pt1, pt2, color=(255,0,0), thickness=5)
    cv2.rectangle(self.hsv_image, pt1, pt2, color=(255,0,0), thickness=5)

    # show images
    cv2.imshow('video_window', self.cv_image)
    cv2.imshow('HSV image', self.hsv_image)    
    cv2.imshow('BW image', self.bw_image)


  def react(self, direction):
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
    print 'direction: ' , ((self.command.linear.x, self.command.angular.z))

  def find_line(self, binary_image):
    """ Given the bw image, track the line and move the bot appropriately. """
    pass

  def stop(self):
    ''' stop all bot motion '''
    self.command.linear.x = 0
    self.command.angular.z = 0

  def send(self):
    self.pub.publish(self.command)

def tr_to_xy(pair):
  ''' convert a theta, radius pair to an x, y pair '''
  angle, radius = pair[0], pair[1]
  x = radius * math.cos(math.radians(angle))
  y = radius * math.sin(math.radians(angle))
  return [x,y]

def xy_to_tr(pair):
  ''' convert an x, y pair to a theta, radius pair '''
  x, y = pair[0], pair[1]
  theta = math.degrees(math.atan((y / x)))
  radius = math.sqrt(x ** 2 + y ** 2)
  return [theta, radius]

def distance(pt1, pt2):
  pt1 = np.array((pt1[0], pt1[1]))
  pt2 = np.array((pt2[0], pt2[1]))
  return np.linalg.norm(pt1-pt2)

def average_two_points(pt1, pt2):
  return [((pt1[0] + pt2[0]) / 2), ((pt1[1] + pt2[1]) / 2)]

def find_two_closest(point_list):
  closest_indices = []
  smallest_distance = 100
  for i in range(len(point_list)):
    for j in range(len(point_list)):
      dist = distance(point_list[i], point_list[j])
      if dist < smallest_distance:
        closest_indices = [i, j]
        smallest_distance = dist
  return [point_list[closest_indices[0]],point_list[closest_indices[1]]]

controller = Controller()

while not rospy.is_shutdown():
  # controller.update_command()
  controller.send()
