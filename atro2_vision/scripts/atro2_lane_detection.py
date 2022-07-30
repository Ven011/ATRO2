#! /usr/bin/python3

"""
    This script is used to precisely distinguish lanes in an image.
"""

import cv2
import numpy as np
import math
import rospy
import copy
from cv_bridge import CvBridge
from atro2_vision.srv import getImage
from atro2_vision.msg import lane
from time import monotonic

class Lane_detector:
    def __init__(self):
        self.test_img = cv2.imread("lane_sample4.jpg")

    def prep_image(self, image, interest_region: list):
        # resize the image
        image = cv2.resize(image, (800, 800))
        img_h = image.shape[0]
        img_w = image.shape[1]
        # crop the image based on the interest region
        image = image[interest_region[0]:interest_region[1], :]

        # blur the image to include more tape pixels in the mask
        image = cv2.GaussianBlur(image, (3, 3), 1)
        # Mask the image to isolate the tape
        low_blue = np.array([70, 70, 102])
        high_blue = np.array([248, 248, 255])
            # convert BGR image to HSV image
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            # extract a mask
        mask = cv2.inRange(hsv, low_blue, high_blue)

        return image, mask

    def scan(self, image, mask, lane_resolution: int = 1):
        img_w = image.shape[1]
        # scan the image and fill the left and right lane point arrays
        lane_resolution_c = 0
        l_lane_l = list()
        l_lane_r = list()
        r_lane_r = list()
        r_lane_l = list()
        c = 0
        for mask_row in mask:
            if not lane_resolution_c % lane_resolution:
                # scan left
                lll_found = False   # left lane left point
                for idx in range(0, img_w - 1):
                    if not lll_found:
                        if mask_row[idx]:
                            l_lane_l.append([idx, lane_resolution_c]) # append the l_lane_l list with the point of the detected lane point
                            lll_found = True
                        elif idx == img_w - 2:
                            l_lane_l.append([0, lane_resolution_c])   # nothing was found append a 0
                            l_lane_r.append([0, lane_resolution_c])
                    if lll_found:   
                        if not mask_row[idx]:
                            l_lane_r.append([idx, lane_resolution_c])
                            break
                        elif idx == img_w - 2:
                            l_lane_r.append(l_lane_l[len(l_lane_l) - 1])    # set the right side of the left lane to the left side if the right side 
                                                                            # could not be found
                    
                # scan right
                rlr_found = False   # right lane right point
                for idx in range(img_w - 1, -1, -1):
                    if not rlr_found:
                        if mask_row[idx]:
                            r_lane_r.append([idx, lane_resolution_c]) # append the l_lane_l list with the point of the detected lane point
                            rlr_found = True
                        elif idx == 0:
                            r_lane_r.append([img_w - 1, lane_resolution_c])   # nothing was found append the width of the image
                            r_lane_l.append([img_w - 1, lane_resolution_c])
                    if rlr_found:   
                        if not mask_row[idx]:
                            r_lane_l.append([idx, lane_resolution_c])
                            break
                        elif idx == 0:
                            r_lane_l.append(r_lane_r[len(r_lane_r) - 1])    # set the right side of the left lane to the left side if the right side 
                                                                            # could not be found
            lane_resolution_c += 1

        cv2.imshow("mask", mask)

        return l_lane_l, l_lane_r, r_lane_l, r_lane_r

    def skinny_lane(self, lll: list, llr: list, rll: list, rlr: list):
        # find the midpoint of each lane's points
        skinny_left = [[(lll[idx][0] + llr[idx][0]) // 2, lll[idx][1]] for idx in range(len(lll) - 1)]
        skinny_right = [[(rll[idx][0] + rlr[idx][0]) // 2, rll[idx][1]] for idx in range(len(rll) - 1)]

        return skinny_left, skinny_right

    def clean_lanes(self, img, skinny_left: list, skinny_right: list, dispute_severity: int = 10, ownership_range: int = 20):
        # resolve disputes (both lanes have the same point)
        
        # questionable (may not belong in the lane) lane points based on distance relative to other lane points
        qr_points = ['g' for _ in range(len(skinny_right))] # g for good
        ql_points = ['g' for _ in range(len(skinny_left))]

        # disputed points - points claimed by the left and right lane
        d_points = [0 for _ in range(len(skinny_right))]

        # find the disputed points
        idx = 0
        for point in skinny_left:
            dist = math.sqrt(math.pow((point[0] - skinny_right[idx][0]), 2) + math.pow((point[1] - skinny_right[idx][1]), 2))
            if dist <= dispute_severity:
                d_points[idx] = 1
            idx += 1

        # find the distance between each point of the lanes
        sl_dist = list()
        sr_dist = list()

        # left lane
        start_point = skinny_left[0]
        for point in skinny_left:
            if point != start_point:
                dist = math.sqrt(math.pow((point[0] - start_point[0]), 2) + math.pow((point[1] - start_point[1]), 2))
                sl_dist.append(int(dist))
                start_point = point

        # right lane
        start_point = skinny_right[0]
        for point in skinny_right:
            if point != start_point:
                dist = math.sqrt(math.pow((point[0] - start_point[0]), 2) + math.pow((point[1] - start_point[1]), 2))
                sr_dist.append(int(dist))
                start_point = point

        # mark the questionable lane points
        idx = 0
        for dist in sl_dist:
            if dist > ownership_range:
                ql_points[idx] += 'd'   # d for distance issue
                ql_points[idx + 1] += 'd'
            idx += 1

        idx = 0
        for dist in sr_dist:
            if dist > ownership_range:
                qr_points[idx] += 'd'   # d for distance issue
                qr_points[idx + 1] += 'd'
            idx += 1

        # merge questionable lane points and disputed lane points
        for idx in range(len(ql_points)):
            if d_points[idx]:
                ql_points[idx] += 'o'    # o for ownership issue

        for idx in range(len(qr_points)):
            if d_points[idx]:
                qr_points[idx] += 'o'    # o for ownership issue

        # settle disputes. if the lane has clear ownership of points, declare it as the owner
        # if a 'go' is near a 'g', the lane must be a clear owner of the point
        # iterate each list forwards and backwards: This helps settle disputes when only one lane is visible in the image
        for idx in range(1, len(qr_points) - 1):
            if qr_points[idx] == 'go' and (qr_points[idx - 1] == 'g' or qr_points[idx + 1] == 'g'):
                qr_points[idx] = 'g'

        for idx in range(1, len(ql_points) - 1):
            if ql_points[idx] == 'go' and (ql_points[idx - 1] == 'g' or ql_points[idx + 1] == 'g'):
                ql_points[idx] = 'g'

        # get each lane to disown a point that it claims, yet the other lane has ownership
        for idx in range(len(qr_points)):
            if 'o' in qr_points[idx] and ql_points[idx] == 'g':
                qr_points[idx] = 'x'

        for idx in range(len(ql_points)):
            if 'o' in ql_points[idx] and qr_points[idx] == 'g':
                ql_points[idx] = 'x'

        # redraw the lanes using the ownership status
        for idx in range(len(qr_points)):
            if qr_points[idx] == 'x':
                skinny_right[idx][0] = img.shape[1] - 1

        for idx in range(len(ql_points)):
            if ql_points[idx] == 'x':
                skinny_left[idx][0] = 0
                
        return skinny_left, skinny_right

    def get_path(self, image, ll: list, rl: list):
        # the path is a list of midpoints derived from each point on the left and right lanes
        path = [[(ll[idx][0] + rl[idx][0]) // 2, ll[idx][1]] for idx in range(len(ll) - 1)]

        for point in path:
            cv2.circle(image, tuple(point), 1, (255, 255, 255), 3)

        return image, path

    def lane_heading(self, img, path: list, path_range_percent: list, negate_slope: bool = True):
        if path_range_percent[0] and path_range_percent[1]:
            # turn percent range into path point indexes
            top = int(len(path) * path_range_percent[0]) - 1
            btm = int(len(path) * path_range_percent[1]) - 1

            cv2.circle(img, tuple(path[top]), 1, (0, 0, 255), 2)
            cv2.circle(img, tuple(path[btm]), 1, (0, 0, 255), 2)
            cv2.line(img, tuple(path[top]), tuple(path[btm]), (0, 0, 255), 1)

            # find and return the slope btw the two points selected by the range
            m = 1000    # this will signify a vertical line
            if (path[btm][0] - path[top][0]):
                m = (path[btm][1] - path[top][1]) / (path[btm][0] - path[top][0])
            else:
                pass
            return img, -m if negate_slope else m
        else:
            raise Exception("path range has to be > 0 and <= 1")

    def run(self, image, interest_region: list, lane_resolution: int, show_path: bool = True, dispute_severity: int = 50, ownership_range: int = 30, path_range_percent: list = [0.6, 0.7]):
        s_time = monotonic()
        img, mask = self.prep_image(image, interest_region=interest_region)
        lll, llr, rll, rlr = self.scan(img, mask, lane_resolution=lane_resolution)
        s_left, s_right = self.skinny_lane(lll=lll, llr=llr, rll=rll, rlr=rlr)
        c_left, c_right = self.clean_lanes(skinny_left=s_left, skinny_right=s_right, img=img, dispute_severity=dispute_severity, ownership_range=ownership_range)
        path_img, path = self.get_path(ll=c_left, rl=c_right, image=img)
        path_with_heading, path_heading = self.lane_heading(img=path_img, path=path, path_range_percent=path_range_percent)

        if show_path:
            cv2.imshow("s_img", path_with_heading)
            cv2.waitKey(1)

        return path, path_heading, (monotonic() - s_time)

def main():
    ld = Lane_detector()
    # create node
    rospy.init_node("lane_detector")
    # wait for the get_image service to become available
    rospy.wait_for_service("/image_srv/get_image")
    bridge = CvBridge()

    # publisher for lane path and path heading information
    path_pub = rospy.Publisher("lane_info", lane, queue_size=1)
    path_msg = lane()

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
            
            # run the lane detector and retrieve the path, path heading and total processing time
            path, path_heading, p_time = ld.run(img, interest_region=[0, img_h], lane_resolution=10, path_range_percent=[0.6, 0.7])

            # publish the path, path heading, and processing time
            path_msg.processing_time = p_time
            path_msg.path_heading = path_heading
            path_msg.path_x = [path[i][0] for i in range(len(path))]
            path_msg.path_y = [path[i][1] for i in range(len(path))]
            path_pub.publish(path_msg)

        except rospy.ServiceException:
            print("Service call failed")

    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass




        
