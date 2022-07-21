#! /usr/bin/python3

"""
    This script uses images captures by the camera to keep the robot between
    a lane of tape. 
"""

import rospy
import cv2
import math
import copy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class Line_follower:

    def __init__(self):
        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber("/camera", Image, self.img_callback)
        
        self._low_blue = np.array([60, 70, 0])
        self._high_blue = np.array([207, 161, 146])
        self._k_width = 5
        self._k_height = 5
        self._threshold1 = 80
        self._threshold2 = 85

    def line_detection(self, image):
        # create and apply mask on image to isolate tape by turning all pixels not considered tape black
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self._low_blue, self._high_blue)
        for row in range(image.shape[0]):
            for col in range(image.shape[1]):
                if not mask[row, col]:
                    image[row, col] = [0, 0, 0]

        # turn the image into a gray scale image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # blur the image to get rid of noise
        blurred = cv2.GaussianBlur(gray, (self._k_width, self._k_height), 0)

        # use canny edge detection to detect edges in the image
        edged = cv2.Canny(blurred, self._threshold1, self._threshold2)

        # use the probabilistic hough transform to extract lines from the image
        lines = cv2.HoughLinesP(edged, 1, np.pi/180, 10, minLineLength=25, maxLineGap=5)

        # draw the lines on the image
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(image, (x1, y1), (x2, y2), (255, 255, 255), 2)

        cv2.imshow("Image", image)
        cv2.waitKey(1)


    def img_callback(self, img_msg):
        try:
            image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)

        img = copy.deepcopy(image)
        self.line_detection(img)



def main():
    lf = Line_follower()
    
    # create node
    rospy.init_node("line_following")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")

    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
