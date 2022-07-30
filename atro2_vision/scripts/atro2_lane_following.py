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
from std_msgs.msg import String
from atro2_vision.msg import lane

class lane_follower:
    def __init__(self):
        # publisher to the robot control topic
        self.cmd_pub = rospy.Publisher("move_cmds", String, queue_size=1)
        
        # subscriber to lane detector topic
        rospy.Subscriber("/lane_info", lane, self.lane_sub_clb)
        # variables to be updated when lane info topic published info
        self.path_p_time = 0    # time is took to derive the path from an image
        self.path = list()
        self.path_heading = 0

        # publisher message
        self.cmd_msg = String()

        # setup timer
        self.curr_time = 0

    def run(self):
        while not rospy.is_shutdown():
            print("p_time: {}   duty: {}".format(self.path_p_time, self.get_duty_cycle(self.path_heading)))

    def get_duty_cycle(self, heading):
        # turn heading to movement duty cycle
        # if heading / path slope is large (almost vertical) the turn duty cycle will be low
        # if the heading is small (almost horizontal) the turn duty cycle will be high
        # The direction to turn is determined by the sign of the heading and is preserved after
        # the calculation
        duty = 0    # signify that no valid heading has been received
        if heading:
            duty = 1 / heading
        return duty

    def lane_sub_clb(self, msg):
        self.path_p_time = msg.processing_time
        self.path = [[msg.path_x[i], msg.path_y[i]] for i in range(len(msg.path_x))]
        self.path_heading = msg.path_heading

def main():
    rospy.init_node("lane_follower")
    lf = lane_follower()
    # run the lane_follower
    lf.run()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
