- For the camera frame publisher to work correctly:
    - Add cv_bridge as a package dependency
    - Install cv_bridge for the ROS distro (If not already installed):
        sudo apt-get install ros-noetic-cv-bridge

- You can view the captured frames in a topic using rqt_image_view:
    - In terminal, type:
        rqt_image_view
        - And select the topic that contrains the image message
