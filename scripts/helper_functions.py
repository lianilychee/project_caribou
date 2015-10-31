#!/usr/bin/env python

import math
import numpy as np
import cv2

def find_line(image, top_left, bottom_right, lower_bound, upper_bound, threshold):
  """
  Given the bw image, define the bounding box, find the line, and output the appropriate heading. 
  Coordinates in (row,col) aka (x,y)
  """

  binary_image_cropped = cv2.inRange(
    image[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0]], 
    lower_bound, upper_bound)
  cv2.imshow('bw_window_cropped', binary_image_cropped) #TODO: UNCOMMENT

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