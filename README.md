# Waypoint Navigation on Unitree Go1 Edu in Irregular Outdoor Terrains

This Repo has 3 pipelines to waypoint autonomous navigation on the Unitree Go1 Edu version. To install this on your system you have to follow this steps:

1 -> Install docker on you machine:

Official docker website: https://docs.docker.com/desktop/

OR

    Ubuntu:
        sudo apt install docker docker-compose
    
    Fedora:
        sudo dnf install docker docker-compose

2 -> Clone this repository to you machine:

2.1 -> Install git:

    Ubuntu:
        sudo apt install git
    Fedora:
        sudo dnf install git
        
2.2 -> Clone Repo

        git clone https://github.com/Gostimari/go1_repo.git

3 -> Enter the docker directory of this cloned repository and run docker compose: (by default is elevation_mapping that will run, you can change the launch file in the docker-compose.yml file)

Run this command to enable docker with GUI:

    xhost +local:root

Official docker website:

    docker compose up
    
Ubuntu/Fedora:

    docker-compose up

Now that all the packages and workspace is ready you have three options to start the system. Every options has a different mapping algorithm.

Navigation with elevation_mapping:

ROS NOETIC:

    roslaunch ig_lio noetic_main_elev.launch

Navigation with Mechanical Effort Based Traversability:

ROS NOETIC:

    roslaunch ig_lio noetic_main_mebt.launch

Navigation with traversability_mapping:

ROS NOETIC:

    roslaunch ig_lio noetic_main_trav.launch

ROS MELODIC:

    roslaunch traversability_mapping offline.launch


To run individually all the algorithms you have bellow the launch files to the different packages:

ig_lio:
    
ROS NOETIC:

    roslaunch ig_lio lio_velodyne_Bpearl.launch

traversability_mapping:

ROS MELODIC:

    roslaunch traversability_mapping offline.launch


elevation_mapping:

ROS NOETIC:

    roslaunch elevation_mapping_demos go1_elevation.launch
    
navigation_final_semfire_pilot:

ROS NOETIC:

    roslaunch navigation_final_semfire_pilot ranger_navigation.launch
    
gps_waypoint_nav:

ROS NOETIC:

COLLECT POINTS:

    roslaunch gps_waypoint_nav collect_goals.launch
    
SEND POINS TO NAVIGATION:

    roslaunch gps_waypoint_nav gps_waypoint_nav.launch
    
MAPVIZ:

    roslaunch gps_waypoint_nav mapviz.launch

MAP DOCKER LOCAL:

You can download map data (PBF files) from download.geofabrik.de. For example, to download data for Portugal, run this command:
    
    wget 'https://download.geofabrik.de/europe/portugal-latest.osm.pbf'

Once we have the data, we need to import it into the tile server. Run these commands to import it to the docker:

    docker volume create openstreetmap-data
    docker volume create openstreetmap-rendered-tiles
    docker run \
        -v $HOME/portugal-latest.osm.pbf:/data.osm.pbf \
        -v openstreetmap-data:/var/lib/postgresql/12/main \
        -v openstreetmap-rendered-tiles:/var/lib/mod_tile \
        -e THREADS=24 \
        overv/openstreetmap-tile-server:1.3.10 \
    import
 
 Once the data has been imported, you can run your server like this:
 
    docker run \
        -p 8080:80 \
        -v openstreetmap-data:/var/lib/postgresql/12/main \
        -v openstreetmap-rendered-tiles:/var/lib/mod_tile \
        -e THREADS=24 \
        -e ALLOW_CORS=enabled \
        -d overv/openstreetmap-tile-server:1.3.10 \
        run
  
MAPVIZ TILE-MAP LINK:

    http://localhost:8080/wmts/gm_layer/gm_grid/{level}/{x}/{y}.png
    Max Zoom: 19

    or
    
    http://127.0.0.1:8080/tile/{level}/{x}/{y}.png
    Max Zoom: 19
    
metrics_extractor: 
    
1 -> extract metrics from the ros-bag or live and save them into a file:
    
    rosrun metrics_extractor metrics.py
    
2 -> Create plots about the extracted data and save them in ../plot directory: (navigate to /logfiles and change the name of the file)
    
    python3 metrics_analyser_deep.py logfile_metrics-2025-02-03-14-53.csv -o ../plots
        
3-> Genrate a Latex table with the extracted data:

    python3 table_generator.py
    

