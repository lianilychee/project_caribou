#!/usr/bin/env python

import math
import numpy as np
import cv2

def find_line(image, top_left, bottom_right, lower_bound, upper_bound):
    """ Given the bw image, define the bounding box, find the line, and output the appropriate heading. """
    binary_image = cv2.inRange(image, lower_bound, upper_bound)