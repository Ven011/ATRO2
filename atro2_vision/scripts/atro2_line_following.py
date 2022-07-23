#! /usr/bin/python3

"""
    This script uses images captures by the camera to keep the robot between
    a lane of tape. 
"""

from time import monotonic
import rospy
import cv2
import math
import copy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from atro2_vision.srv import getImage

class Line_follower:

    def __init__(self):
        self.bridge = CvBridge()
        
        self._low_blue = np.array([60, 70, 0])
        self._high_blue = np.array([207, 161, 146])
        self._k_width = 5
        self._k_height = 5
        self._threshold1 = 80
        self._threshold2 = 85

    def line_detection(self, image):
        s_time = monotonic()
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
        lines = cv2.HoughLinesP(edged, 1, np.pi/180, 20, minLineLength=15, maxLineGap=5)

        # draw the lines on the image
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(image, (x1, y1), (x2, y2), (255, 255, 255), 2)

        print("Processing time: {} sec".format(round(monotonic() - s_time, 5)))

        return image

    def show_results(self, ros_img):
        cv2_img = copy.deepcopy(ros_img)
        cv2_img = self.bridge.imgmsg_to_cv2(cv2_img, "bgr8")

        processed_img = self.line_detection(cv2_img)

        cv2.imshow("image", processed_img)
        cv2.waitKey(1)

def main():
    lf = Line_follower()
    # create node
    rospy.init_node("line_following")
    # wait for the get_image service to become available
    rospy.wait_for_service("/image_srv/get_image")

    while not rospy.is_shutdown():
        # request an image and show the results after processing
        try:
            # create a service call proxy: Its used to make a call to the service
            srv_call = rospy.ServiceProxy("/image_srv/get_image", getImage)
            # make a call to the service with the given arguments and take the response
            srv_resp = srv_call(True)
            # get the image from the response and show the results after processing
            lf.show_results(srv_resp.cap_image)
        except rospy.ServiceException:
            print("Service call failed")

    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
