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
        self.curr_time = monotonic()

        self.turn_freq_gain = 0.5

    def run(self):
        while not rospy.is_shutdown():
            # depending on the turn frequency and turn direction, publish the appropriate control message
            turn_freq = self.get_frequency()
            print(turn_freq)
            if monotonic() - self.curr_time >= abs(turn_freq):
                self.curr_time = monotonic()
                if turn_freq > 0:
                    self.cmd_msg.data = "r"
                elif turn_freq < 0:
                    self.cmd_msg.data = "l"
                self.cmd_pub.publish(self.cmd_msg)
            else:
                self.cmd_msg.data = "f"
                self.cmd_pub.publish(self.cmd_msg)

    def get_frequency(self):
        # turn heading into a frequency value
        # if heading / path slope is large (almost vertical) the turning frequency will be low
        # if the heading is small (almost horizontal) the turning frequency will be high
        # The direction to turn is determined by the sign of the heading and is preserved after
        # the calculation
        freq = 0    # signify that no valid heading has been received
        if self.path_heading:
            freq = self.path_heading * self.turn_freq_gain
        return freq

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
