#!/usr/bin/env python

import math
import numpy as np
import cv2

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