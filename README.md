# go1_repo

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/duarte/testes-noetic/src/ros_unitree/unitree_ros/unitree_gazebo/worlds/models

Melodic compilation:
    catkin_make -DCATKIN_WHITELIST_PACKAGES="cloud_msgs"
    catkin_make -DCATKIN_WHITELIST_PACKAGES=""

Traversability Mapping:

ROS NOETIC:

    roslaunch unitree_gazebo robot_simulation.launch

    rosrun unitree_guide junior_ctrl   ( tecla 2 -> 5 para ficar em move_base)

ROS MELODIC:

    roslaunch traversability_mapping offline_sim.launch


Elevation Mapping:

ROS NOETIC:

    roslaunch unitree_gazebo elev_simulation.launch

    rosrun unitree_guide junior_ctrl   ( tecla 2 -> 5 para ficar em move_base)

    roslaunch elevation_mapping_demos go1_elevation_sim.launch
    
MEBT:
    roslaunch navigation_final_semfire_pilot ranger_navigation.launch


Waypoint Navigation on Traversability e Elevation:

ROS NOETIC:

    - Change the desired waypoints on the .txt file located on: /outdoor_waypoint_nav/waypoint_files/points_sim.txt

    roslaunch outdoor_waypoint_nav send_goals_sim.launch


SLAM com OCTOMAP: (Keyboard Navigation)

    roslaunch unitree_gazebo slam_simulation.launch

    rosrun unitree_guide junior_ctrl   ( tecla 2 -> 5 para ficar em move_base)

    roslaunch unitree_navigation slam.launch
    
LIO-SAM:
    
ROS MELODIC:

    roslaunch lio_sam run.launch

ROS NOETIC:

    IG-LIO:
    roslaunch ig_lio lio_velodyne_Bpearl.launch

gps_waypoint_nav:

ROS NOETIC:

    COLLECT POINTS:
    roslaunch gps_waypoint_nav collect_goals.launch
    
    roslaunch gps_waypoint_nav gps_waypoint_nav.launch
    
    MAPVIZ:
    roslaunch gps_waypoint_nav mapviz.launch


    
MAP DOCKER:

    sudo docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
    
    or Docker Desktop App
    
MAPVIZ TILE-MAP LINK:
    
    http://localhost:8080/wmts/gm_layer/gm_grid/{level}/{x}/{y}.png
    Max Zoom: 19
    
    
Bag: (IMU-GNSS)
    
    rosbag play bagfile_*.bag gt.bag --clock
