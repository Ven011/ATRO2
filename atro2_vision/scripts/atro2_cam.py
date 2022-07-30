#! /usr/bin/python3

"""
    This script operates an image service that provides
    captured frames to other nodes when requested
"""

import cv2
import rospy
from cv_bridge import CvBridge
from atro2_vision.srv import getImage, getImageResponse

class Image_srv:
    def __init__(self):
        # service node
        rospy.init_node("image_srv")

        # image capture object
        self.cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)

        # service server
        s = rospy.Service("/image_srv/get_image", getImage, self.img_srv_callback)

        self.bridge = CvBridge()

    def __del__(self):
        self.cap.release()

    def img_srv_callback(self, req):
        # capture and return image
        ret, img = self.cap.read()
        img = cv2.resize(img, (300, 300))
        #img = cv2.flip(img, 0)
        ros_img = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        return getImageResponse(ros_img)

if __name__ == "__main__":
    try: 
        img_srv = Image_srv()
        rospy.spin()
        del img_srv
    except rospy.ROSInterruptException:
        pass
