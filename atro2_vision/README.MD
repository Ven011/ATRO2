- For the camera frame publisher to work correctly:
    - Add cv_bridge as a package dependency
    - Install cv_bridge for the ROS distro (If not already installed):
        sudo apt-get install ros-noetic-cv-bridge

- You can view the captured frames in a topic using rqt_image_view:
    - In terminal, type:
        rqt_image_view
        - And select the topic that contrains the image message

- When creating a service message, you can use any rosmsg (std_msgs, sensor_msgs, etc.)
    - Just make sure to add the package in the CMakeLists find package function:
        find_package(catkin REQUIRED COMPONENTS
            roscpp
            rospy
            std_msgs
            sensor_msgs
            message_generation
        )
    - and the message generation function:
        generate_messages(
            DEPENDENCIES
            std_msgs  # Or other packages containing msgs
            sensor_msgs
        )
