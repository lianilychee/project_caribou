#!/usr/bin/env python

import math
import numpy as np
import cv2
from matplotlib import pyplot as plt
from scipy.linalg import norm
from scipy import sum, average
import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Vector3
import match_keypoints as mk
import rospkg

<<<<<<< HEAD


=======
>>>>>>> 778f12d8058557b7de32b47dab821c266dacbbb1
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


def find_boundary_pts():
  """
  Given the bw image, find the boundary points, and output the min/max points 
  needed to draw a two-point rectangle.
  """

<<<<<<< HEAD
  image = cv2.imread('../stop.png')
  gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
  gray = cv2.GaussianBlur(gray, (3, 3), 0)
  #cv2.imshow("Gray",gray)
  #cv2.waitKey(0)
  edged = cv2.Canny(gray,10,250)
  #cv2.imshow("Edged", edged)
  #cv2.waitKey(0)
  kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
  closed = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel)
  #cv2.imshow("Closed",closed)
  #cv2.waitKey(0)
  (cnts, _) = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  total = 0

  for c in cnts:
    peri = cv2.arcLength(c,True)
    approx = cv2.approxPolyDP(c, 0.02 * peri, True)

    if len(approx) == 8:
      cv2.drawContours(image, [approx], -1, (0, 255, 0), 4)
      total +=1

  cv2.imshow("Output", image)
  cv2.waitKey(0)
  
=======
  im = cv2.imread('../test_images/stop.png')

  im_gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

  ret,thresh = cv2.threshold(im_gray,127,255,0)
  contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) 

  cv2.drawContours(im, contours, -1, (0,255,0), 3)

  # print contours[6].shape

  x_list = []
  y_list = []

  for contour in contours:
    # for pt in np.nditer(contour):
    for x,y in contour.reshape(contour.shape[0], contour.shape[2]):
      x_list.append(x)
      y_list.append(y)

  pt1 = (min(x_list), min(y_list))
  pt2 = (max(x_list), max(y_list))

  cv2.waitKey(0)

  return pt1, pt2



find_boundary_pts()
>>>>>>> 778f12d8058557b7de32b47dab821c266dacbbb1
