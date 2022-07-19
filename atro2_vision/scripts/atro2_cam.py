#! /usr/bin/python3

"""
    This script creates and publishes a camera topic that provides
    captured frames to other nodes.
"""

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def main():
    # camera capture object
    cam = cv2.VideoCapture(0) # TODO: Change when on raspberry pi. Add cv2.CAP_V4L2
    # camera bridge
    bridge = CvBridge()

    # create node
    rospy.init_node("image")
    # create image publisher
    img_pub = rospy.Publisher("/camera", Image, queue_size=1)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        ret, frame = cam.read()
        if not ret: 
            break   # break out of the loop if we didn't capture an image

        # resize and flip the frame
        frame = cv2.resize(frame, (600, 600))
        frame = cv2.flip(frame, 0)

        # create out topic message using the bridge
        msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        # publish the message
        img_pub.publish(msg)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        if rospy.is_shutdown():    
            cam.release()

if __name__ == "__main__":
    try: 
        main()
    except rospy.ROSInterruptException:
        pass
