#!/usr/bin/env python

import math
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Vector3


def find_line(image,top_left,bottom_right,lower_bound,upper_bound,threshold):
  """
  Given the bw image, define the cropped image, find the line, and output the 
  appropriate heading. 
  Coordinates in (row,col) aka (x,y)
  """
  binary_image_cropped = cv2.inRange(
    image[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0]], 
    lower_bound, upper_bound)
  cv2.imshow('bw_window_cropped', binary_image_cropped)

  centerline = (bottom_right[0] - top_left[0]) / 2

  count = 0
  sum_col = 0

  for row in range(binary_image_cropped.shape[0]):
    for col in range(binary_image_cropped.shape[1]):
      if binary_image_cropped[row,col]:
        count += 1
        sum_col += col
  if count < 10:
    return (0, False)

  avg_col = sum_col / count
  
  if (avg_col - centerline) > threshold:
    return (-(avg_col - centerline), True) # turn right
  elif (avg_col - centerline) < -threshold:
    return (-(avg_col - centerline), True) # turn left
  else:
    return (0, True) # line could be nonexistent or centered


def find_stop_sign(image, lb, ub):
  """
  Find the octagon by:
  changing raw image colorspace from BGR to HSV,
  filtering out red from HSV image,
  changing filtered image to greyscale,
  performing Gaussian blur and canny edge detection
  """

  image = image

  hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
  bw_image = cv2.inRange(hsv_image, lb, ub) 

  print 'lb: ', lb, '     ub: ', ub

  cv2.imshow("bw img", bw_image)

  gray_image = bw_image
  # gray_image = cv2.cvtColor(bw_image, cv2.COLOR_BGR2GRAY)

  gray_image = cv2.GaussianBlur(gray_image, (3, 3), 0)
  # cv2.imshow("Gray",gray_image) #UNCOMMENT
  edged = cv2.Canny(gray_image,10,250)
  #cv2.imshow("Edged", edged)
  kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
  closed = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel)
  #cv2.imshow("Closed",closed)
  #cv2.waitKey(0)
  (cnts, _) = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  total = 0
  for c in cnts:
    peri = cv2.arcLength(c,True)
    approx = cv2.approxPolyDP(c, 0.02 * peri, True)
    print cv2.contourArea(approx)
    if len(approx) == 8 and cv2.contourArea(c) > 10000:
      cv2.drawContours(image, [approx], -1, (0, 255, 0), 4)
      total +=1

  cv2.imshow("Output", image)
  cv2.waitKey(5)

  return (total > 0)

  


# find_stop_sign(cv2.imread('../test_images/live.png'), (0,188,42), (255,255,255))