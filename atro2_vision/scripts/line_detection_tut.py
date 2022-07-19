#! /usr/bin/python3

import cv2
import numpy as np
import math

image = cv2.imread("lane_sample.jpg")

# resize the image
image = cv2.resize(image, (600, 600))

# Mask the image to isolate the tape
low_blue = np.array([60, 70, 0])
high_blue = np.array([207, 161, 146])
    # convert BGR image to HSV image
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # extract a mask
mask = cv2.inRange(hsv, low_blue, high_blue)

# cv2.imshow("Masked image", mask)

# turn all pixels that are not considered tape black
for row in range(image.shape[0]):
    for col in range(image.shape[1]):
        if not mask[row, col]:
            image[row, col] = [0, 0, 0]

# convert the BGR image to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

cv2.imshow("Gray", gray)

# smooth the image to reduce noise using the Gaussian kernel
k_width = 5 # kernel width and height
k_height = 5
blurred = cv2.GaussianBlur(gray, (k_width, k_height), 0)

cv2.imshow("Blurred", blurred)

# edge detection using canny edge detection
threshold1 = 80
threshold2 = 85
edged = cv2.Canny(blurred, threshold1, threshold2)

cv2.imshow("Edges", edged)

# Use the Hought transform to extract shapes (lines) in the image in the form of line coordinates
lines = cv2.HoughLinesP(edged, 1, np.pi/180, 10, minLineLength=50, maxLineGap=10)
for line in lines:
    x1, y1, x2, y2 = line[0]
    cv2.line(image, (x1, y1), (x2, y2), (255, 255, 255), 2)

cv2.imshow("Image", image)

cv2.waitKey(0)

cv2.destroyAllWindows()