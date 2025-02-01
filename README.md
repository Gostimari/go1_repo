# Waypoint Navigation on Unitree Go1 Edu in Irregular Outdoor Terrains

This Repo has 3 pipelines to waypoint autonomous navigation on the Unitree Go1 Edu version. To install this on your system you have to follow this steps:

1 -> Install docker on you machine:

    Official docker website: [docs.docker.com](https://docs.docker.com/desktop/)

    OR

    Ubuntu:
        ```bash
        sudo apt install docker docker-compose
        ```
    Fedora:
        ```bash
        sudo dnf install docker docker-compose
        ```
2 -> Clone this repository to you machine:

    2.1 -> Install git:
            Ubuntu:
                ```bash
                sudo apt install git
                ```
            Fedora:
                ```bash
                sudo dnf install git
                ```
    2.2 -> Clone Repo
            ```bash
            git clone https://github.com/Gostimari/go1_repo.git
            ```

3 -> Enter the docker directory of this cloned repository and run docker compose:

    Official docker website:
        ```bash
        docker compose up
        ```
    
    Ubuntu/Fedora:
        ```bash
        docker-compose up
        ```

Now that all the packages and workspace is ready you have three options to start the system. Every options has a different mapping algorithm.

Navigation with elevation_mapping:

    ROS NOETIC:
    ```bash
    roslaunch ig_lio noetic_main_elev.launch
    ```

Navigation with Mechanical Effort Based Traversability:

    ROS NOETIC:
    ```bash
    roslaunch ig_lio noetic_main_mebt.launch
    ```
Navigation with traversability_mapping:

    ROS NOETIC:
    ```bash
    roslaunch ig_lio noetic_main_trav.launch
    ```
    ROS MELODIC:
    ```bash
    roslaunch traversability_mapping offline.launch
    ```

To run individually all the algorithms you have bellow the launch files to the different packages:

ig_lio:
    
    ROS NOETIC:
    ```bash
    roslaunch ig_lio lio_velodyne_Bpearl.launch
    ```
traversability_mapping:

    ROS MELODIC:
    ```bash
    roslaunch traversability_mapping offline.launch
    ```

elevation_mapping:

    ROS NOETIC:
    ```bash
    roslaunch elevation_mapping_demos go1_elevation.launch
    ```
navigation_final_semfire_pilot:

    ROS NOETIC:
    ```bash
    roslaunch navigation_final_semfire_pilot ranger_navigation.launch
    ```
gps_waypoint_nav:

    ROS NOETIC:

        COLLECT POINTS:
        ```bash
        roslaunch gps_waypoint_nav collect_goals.launch
        ```
        SEND POINS TO NAVIGATION:
        ```bash
        roslaunch gps_waypoint_nav gps_waypoint_nav.launch
        ```
        MAPVIZ:
        ```bash
        roslaunch gps_waypoint_nav mapviz.launch
        ```
        MAP DOCKER:
        ```bash
        sudo docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
        ```  
        or 
        Docker Official Desktop App
        
        MAPVIZ TILE-MAP LINK:
        http://localhost:8080/wmts/gm_layer/gm_grid/{level}/{x}/{y}.png
        Max Zoom: 19
