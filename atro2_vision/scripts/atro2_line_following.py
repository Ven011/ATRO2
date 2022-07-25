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
from std_msgs.msg import String
from atro2_vision.srv import getImage

class Line_follower:

    def __init__(self):
        self.bridge = CvBridge()
        
        self._low_blue = np.array([60, 70, 0])
        self._high_blue = np.array([207, 161, 146])
        self._k_width = 3
        self._k_height = 3
        self._threshold1 = 80
        self._threshold2 = 85

        self.turn_offset = 0.3  # offset used to determine whether we should turn left or right
        self.inv_turn = False
        # turning control topic publisher
        self.cmd_pub = rospy.Publisher("move_cmds", String, queue_size=1)
        self.cmd_msg = String()

    def line_detection(self, image):
        s_time = monotonic()
        # create and apply mask on image to isolate tape by turning all pixels not considered tape black
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self._low_blue, self._high_blue)
        for row in range(image.shape[0]):
            for col in range(image.shape[1]):
                if not mask[row, col]:
                    image[row, col] = [0, 0, 0]

        cv2.imshow("mask", mask)

        # turn the image into a gray scale image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # blur the image to get rid of noise
        blurred = cv2.GaussianBlur(gray, (self._k_width, self._k_height), 0)

        # use canny edge detection to detect edges in the image
        edged = cv2.Canny(blurred, self._threshold1, self._threshold2)

        # use the probabilistic hough transform to extract lines from the image
        lines = cv2.HoughLinesP(edged, 1, np.pi/180, 20, minLineLength=15, maxLineGap=5)

        return lines, image, (monotonic() - s_time)

    def line_point_detection(self, image):
        s_time = monotonic()
        # blur the image to get rid of noise
        image = cv2.GaussianBlur(image, (self._k_width, self._k_height), 6)
        # Mask the image to isolate the tape
        low_blue = np.array([60, 70, 0])
        high_blue = np.array([207, 161, 146])
            # convert BGR image to HSV image
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            # extract a mask
        mask = cv2.inRange(hsv, low_blue, high_blue)

        # create a path array - stores the points of the center path between the lanes
        path_interval = 1  # path resolution in pixels

            # append the path array with the points of a line that vertically spans the center of the image
        o_px = image.shape[1] // 2  # origint point coords. bottom center pixel of the image
        o_py = image.shape[0]
        path = [[o_px, y] for y in range(o_py, 0, -path_interval)]

        # set each point in the path to the coordinate of the pixel that is in the middle of two
        # white "pixels" that share a row with the point in the mask image. If there is no white pixel,
        # keep the original point.
        path_idx = 0
        for point in path:
            path_idx += 1
            p_row = mask[point[1] - 1]  # subtract one bc indexing starts at 0
            # get the row of pixels left of the point
            l_row = p_row[:len(p_row)//2]   # * could be problematic if the image size is odd
            # get the row of pxls right
            r_row = p_row[len(p_row)//2:]

            # flip the l_row array
            l_row = l_row[::-1]

            # find the index of the first occurence of a 255 in the l and r rows.
            l_white_px = 0
            r_white_px = 0
            for idx in range(len(l_row) - 1):
                if l_row[idx]:
                    l_white_px = idx
                    break

            for idx in range(len(r_row) - 1):
                if r_row[idx]:
                    r_white_px = idx
                    break

            # assume equal lane width if the left or right lane is not detected
            # l_white_px = r_white_px if not l_white_px and r_white_px else l_white_px
            # r_white_px = l_white_px if not r_white_px and l_white_px else r_white_px

            # calculate the mid-point
            if l_white_px and r_white_px:    
                point[0] = ((point[0] - l_white_px) + (point[0] + r_white_px)) // 2
            elif l_white_px:
                point[0] = point[0] + l_white_px
            elif r_white_px:
                point[0] = point[0] - r_white_px
            else:
                point[0] = path[path_idx - 1][0] if path_idx > 1 and path_idx != len(path) else point[0]

            cv2.circle(image, (point[0], point[1]), 1, (0, 0, 255), 1)

            if point[0] != point[0] - l_white_px and point[0] != point[0] + r_white_px:
                cv2.circle(image, (point[0] - l_white_px, point[1]), 1, (255, 255, 255), 1)
                cv2.circle(image, (point[0] + r_white_px, point[1]), 1, (255, 255, 255), 1)

        return image, (monotonic() - s_time)

    def viz_lines(self, lines, image):
        s_time = monotonic()
        # draw the lines on the image
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(image, (x1, y1), (x2, y2), (255, 255, 255), 2)

        return image, (monotonic() - s_time)

    def avg_slope(self, lines):
        s_time = monotonic()
        # determine the slope of each line and calculate the average slope
        total_m = 0
        num_lines = 0
        if lines is not None:
            for line in lines:
                num_lines += 1
                x1, y1, x2, y2 = line[0]
                total_m += (y1 - y2) / (x1 - x2)

        avg_slope = (total_m/num_lines) if num_lines != 0 else 0

        return avg_slope, (monotonic() - s_time)

    def control(self, avg_slope):
        if avg_slope < -self.turn_offset:
            self.cmd_msg.data = "l" if not self.inv_turn else "r"
        elif avg_slope > self.turn_offset:
            self.cmd_msg.data = "r" if not self.inv_turn else "l"
        else:
            self.cmd_msg.data = "f"

        print("Cmd: {}".format(self.cmd_msg.data))

        self.cmd_pub.publish(self.cmd_msg)

    def show_results(self, ros_img):
        cv2_img = copy.deepcopy(ros_img)
        cv2_img = self.bridge.imgmsg_to_cv2(cv2_img, "bgr8")

        # detected_lines, image, ld_p_time = self.line_detection(cv2_img)
        # marked_img, lv_p_time = self.viz_lines(detected_lines, image)
        # average_slope, as_p_time = self.avg_slope(detected_lines)
        # self.control(average_slope)
        # print("Total processing time: {},   Average_slope: {}".format((ld_p_time + lv_p_time + as_p_time), average_slope))

        marked_img, lpd_p_time = self.line_point_detection(cv2_img)

        print(lpd_p_time)

        cv2.imshow("image", marked_img)
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
