# CMake generated Testfile for 
# Source directory: /root/catkin_ws/src/depends/point_cloud_io
# Build directory: /root/catkin_ws/build/point_cloud_io
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_point_cloud_io_gtest_test_point_cloud_io "/root/catkin_ws/build/point_cloud_io/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/root/catkin_ws/build/point_cloud_io/test_results/point_cloud_io/gtest-test_point_cloud_io.xml" "--return-code" "/root/catkin_ws/devel/.private/point_cloud_io/lib/point_cloud_io/test_point_cloud_io --gtest_output=xml:/root/catkin_ws/build/point_cloud_io/test_results/point_cloud_io/gtest-test_point_cloud_io.xml")
set_tests_properties(_ctest_point_cloud_io_gtest_test_point_cloud_io PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/root/catkin_ws/src/depends/point_cloud_io/CMakeLists.txt;95;catkin_add_gtest;/root/catkin_ws/src/depends/point_cloud_io/CMakeLists.txt;0;")
subdirs("gtest")
