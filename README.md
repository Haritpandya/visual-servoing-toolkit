# visual-servoing-toolkit

Visual servoing codes for matlab and python.
Includes gazebo integration.


note: Currently only Python scipts are supported.

Installation:
just copy the floder in your catkin workspace, do catkin_make and source devel/setup.bash

Running:
1. roslaunch visual_servoing free_cam_aruco_marker.launch
2. in another terminal: roscd visual_servoing/scripts/python/examples/
3. python ibvs_aruco.py


Requires:
libgazebo_ros_openni_kinect
