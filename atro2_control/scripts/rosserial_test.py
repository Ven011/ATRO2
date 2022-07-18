#! /usr/bin/python3

import rospy
from std_msgs.msg import Int8

def test():
    # create a node
    rospy.init_node("rosserial_test")

    # create a publisher
    pub = rospy.Publisher("val_pub", Int8, queue_size=1)

    # keep the node and publisher alive
    rospy.spin()

if __name__ == "__main__":
    try:
        test()
    except rospy.ROSInterruptException:
        print("Closing...")