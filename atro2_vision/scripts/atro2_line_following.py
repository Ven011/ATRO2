#! /usr/bin/python3

"""
    This script uses images captures by the camera to keep the robot between
    a lane of tape. 
"""

import rospy
import cv2
import math
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class Line_follower:

    def __init__(self):
        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber("/camera", Image, self.img_callback)

    def img_callback(self, img_msg):
        try:
            image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Image window", image)

        cv2.waitKey(1)



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