# Vision-based-material-handling-system-on-Husky-ground-robot
Jaco arm (j2n6s300 6-DoF) cartesian waypoint control in ROS (Kinetic)

Dependencies :-
1) roscpp
2) sensor_msgs
3) geometry_msgs
4) kinova_msgs
5) image_transport
6) kinova arm driver packages for ROS kinetic

Steps to run :-
1) Launch the arm driver --> roslaunch kinova_bringup kinova_robot.launch**
2) Launch zed ros wrapper --> roslaunch zed_wrapper zed.launch
3) Run the segmentation code --> rosrun **your_package_name** ros_caffe_zed***
4) Run the arm grasping code --> rosrun **yout_package_name** armJaco

** the default robot type is 6DoF arm (j2n6s300), if you want to use another type, add the arguement kinova_robotType=your_robot_type

*** we have another segmentation code without the use of zed ros wrapper (source code --> ros_caffe_brick.cpp). This has issues with transformation matrices from point clouds to the left camera frame. We suggest the use of zed ros wrapper as these transformations are inherent when point clouds and depth images are published.
