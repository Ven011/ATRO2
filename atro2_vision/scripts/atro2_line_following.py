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
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from atro2_vision.srv import getImage
from atro2_lane_detection import Lane_detector

# class Lane_follower:

#     def __init__(self):
#         self.bridge = CvBridge()

#         self.turn_offset = 0.3  # offset used to determine whether we should turn left or right
#         self.inv_turn = False
#         # turning control topic publisher
#         self.cmd_pub = rospy.Publisher("move_cmds", String, queue_size=1)
#         self.cmd_msg = String()

def main():
    # lf = Lane_follower()
    ld = Lane_detector()
    # create node
    rospy.init_node("line_following")
    # wait for the get_image service to become available
    rospy.wait_for_service("/image_srv/get_image")
    bridge = CvBridge()

    while not rospy.is_shutdown():
        # request an image and show the results after processing
        try:
            # create a service call proxy: Its used to make a call to the service
            srv_call = rospy.ServiceProxy("/image_srv/get_image", getImage)
            # make a call to the service with the given arguments and take the response
            srv_resp = srv_call(True)
            # get the image from the response and show the results after processing
            img = copy.deepcopy(srv_resp.cap_image)
            img = bridge.imgmsg_to_cv2(img, "bgr8")
            img_h = img.shape[0]
            
            # run the lane detector and retrieve the path
            path = ld.run(img, interest_region=[0, img_h], lane_resolution=5)
        except rospy.ServiceException:
            print("Service call failed")

    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
