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

speed_factor = .6
personal_space = .5

class Controller:
	def __init__(self):
		rospy.init_node('caribou')

		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		self.command = Twist()

        rospy.Subscriber(image_topic, Image, self.process_image)

        cv2.setMouseCallback('video_window', self.process_mouse_event)
        cv2.namedWindow('threshold_image')
        cv2.namedWindow('video_window')
        cv2.namedWindow('bw_window')

        self.blue_lower_bound = 0
        cv2.createTrackbar('blue lower bound', 'threshold_image', 0, 255, self.set_blue_lower_bound)
 
        self.blue_upper_bound = 1
        cv2.createTrackbar('blue upper bound', 'threshold_image', 0, 255, self.set_blue_upper_bound)

        self.green_lower_bound = 0
        cv2.createTrackbar('green lower bound', 'threshold_image', 0, 255, self.set_green_lower_bound)
     
        self.green_upper_bound = 1
        cv2.createTrackbar('green upper bound', 'threshold_image', 0, 255, self.set_green_upper_bound)

        self.red_lower_bound = 0
        cv2.createTrackbar('red lower bound', 'threshold_image', 0, 255, self.set_red_lower_bound)

        self.red_upper_bound = 1
        cv2.createTrackbar('red upper bound', 'threshold_image', 0, 255, self.set_red_upper_bound)

		self.stop()
		self.drive()

   def process_mouse_event(self, event, x,y,flags,param):
        """ Process mouse events so that you can see the color values associated
            with a particular pixel in the camera images """
        image_info_window = 255*np.ones((500,500,3))
        cv2.putText(image_info_window,
                    'Color (b=%d,g=%d,r=%d)' % (self.cv_image[ty,x,0], self.cv_image[y,x,1], self.cv_image[y,x,2]),
                    (5,50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0,0,0))
        cv2.imshow('image_info', image_info_window)
        cv2.waitKey(5)

    def set_blue_lower_bound(self, val):
        self.blue_lower_bound = val

    def set_green_lower_bound(self, val):
        self.green_lower_bound = val

    def set_red_lower_bound(self, val):
        self.red_lower_bound = val

    def set_blue_upper_bound(self, val):
        self.blue_upper_bound = val

    def set_green_upper_bound(self, val):
        self.green_upper_bound = val

    def set_red_upper_bound(self, val):
        self.red_upper_bound = val

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # print self.cv_image.shape
        cv2.imshow('video_window', self.cv_image)
        cv2.waitKey(5)

        hp.find_line()
        
        cv2.imshow('bw_window', self.binary_image)


    def find_line(self, binary_image):
    	""" Given the bw image, track the line and move the bot appropriately. """


	def stop(self):
		''' stop all bot motion '''
		self.command.linear.x = 0
		self.pub.publish(self.command)

	def drive(self):
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
	controller.update_command()
	controller.drive()
