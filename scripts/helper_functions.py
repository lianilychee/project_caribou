#!/usr/bin/env python

import math
import numpy as np
import cv2
import pytesseract
try:
  import Image
except ImportError:
  from PIL import Image

def find_line(image, top_left, bottom_right, lower_bound, upper_bound, threshold):
  """
  Given the bw image, define the bounding box, find the line, and output the appropriate heading. 
  Coordinates in (row,col) aka (x,y)
  """
  binary_image = cv2.inRange(image[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0]], lower_bound, upper_bound)
  cv2.imshow('bw_window', binary_image)

  centerline = (bottom_right[0] - top_left[0]) / 2

  count = 0
  sum_col = 0

  for row in range(binary_image.shape[0]):
    for col in range(binary_image.shape[1]):
      if binary_image[row,col]:
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


def extract_text(array):
  array = cv2.cvtColor(array, cv2.COLOR_GRAY2RGB)
  mode = 'RGBA'
  size = 640, 480
  array = array.reshape(array.shape[0]*array.shape[1], array.shape[2])
  if len(array[0]) == 3:
    array = np.c_[array, 255*np.ones((len(array),1), np.uint8)]
  img = Image.frombuffer(mode, size, array.tostring(), 'raw', mode, 0, 1)
  return(pytesseract.image_to_string(img))