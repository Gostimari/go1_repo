# CMake generated Testfile for 
# Source directory: /root/catkin_ws/src/gps_waypoint_nav
# Build directory: /root/catkin_ws/build/gps_waypoint_nav
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_gps_waypoint_nav_roslaunch-check_launch "/root/catkin_ws/build/gps_waypoint_nav/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/root/catkin_ws/build/gps_waypoint_nav/test_results/gps_waypoint_nav/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /root/catkin_ws/build/gps_waypoint_nav/test_results/gps_waypoint_nav" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/root/catkin_ws/build/gps_waypoint_nav/test_results/gps_waypoint_nav/roslaunch-check_launch.xml\" \"/root/catkin_ws/src/gps_waypoint_nav/launch\" ")
set_tests_properties(_ctest_gps_waypoint_nav_roslaunch-check_launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/root/catkin_ws/src/gps_waypoint_nav/CMakeLists.txt;21;roslaunch_add_file_check;/root/catkin_ws/src/gps_waypoint_nav/CMakeLists.txt;0;")
subdirs("gtest")
