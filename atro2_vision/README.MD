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

- When importing custom python scripts, make sure the following is done:
    - A setup script is created in the package directory to point to where all the scripts to be installed are located.
    - The <catkin_python_setup> function is called in the CMakeLists file
    - The python files are defined in the catkin_python_install function with the parent folder's path. i.e. /scripts/pythonfile.py

- ROS only supports 1-dimensional arrays in messages
