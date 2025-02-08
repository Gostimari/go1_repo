Here's the improved README.md ready to copy/paste:

```markdown
# 🚀 Autonomous Waypoint Navigation on Unitree Go1 Edu in Irregular Outdoor Terrains

This repository provides three robust pipelines for autonomous waypoint navigation on the Unitree Go1 Edu robot, tailored for irregular outdoor terrains. Below is a step-by-step guide to set up and utilize the system effectively.

---

## 📦 Installation

### 1. Install Docker
Follow the official Docker installation guide for your OS:  
[https://docs.docker.com/desktop/](https://docs.docker.com/desktop/)

**OR** use package managers:  
```bash
# Ubuntu
sudo apt install docker docker-compose

# Fedora
sudo dnf install docker docker-compose
```

### 2. Clone the Repository
#### Install Git
```bash
# Ubuntu
sudo apt install git

# Fedora
sudo dnf install git
```

#### Clone the Repo
```bash
git clone https://github.com/Gostimari/go1_repo.git
```

---

## 🐳 Docker Setup
1. Enable Docker GUI access:
   ```bash
   xhost +local:root
   ```
2. Navigate to the `docker` directory and start the containers:
   ```bash
   docker-compose up
   ```
   *By default, `elevation_mapping` is launched. Modify `docker-compose.yml` to change the startup configuration.*

---

## 🧭 Navigation Pipelines

### Option 1: Elevation Mapping
```bash
# ROS Noetic
roslaunch ig_lio noetic_main_elev.launch
```

### Option 2: Mechanical Effort-Based Traversability (MEBT)
```bash
# ROS Noetic
roslaunch ig_lio noetic_main_mebt.launch
```

### Option 3: Traversability Mapping
```bash
# ROS Noetic
roslaunch ig_lio noetic_main_trav.launch

# ROS Melodic
roslaunch traversability_mapping offline.launch
```

---

## 🛠️ Launch Individual Components

### LIDAR-Inertial Odometry (LIO)
```bash
roslaunch ig_lio lio_velodyne_Bpearl.launch
```

### Elevation Mapping Demos
```bash
roslaunch elevation_mapping_demos go1_elevation.launch
```

### Navigation Stack
```bash
roslaunch navigation_final_semfire_pilot ranger_navigation.launch
```

---

## 🗺️ GPS Waypoint Navigation

### Collect Waypoints
```bash
roslaunch gps_waypoint_nav collect_goals.launch
```

### Start Navigation
```bash
roslaunch gps_waypoint_nav gps_waypoint_nav.launch
```

### Visualize with MapViz
```bash
roslaunch gps_waypoint_nav mapviz.launch
```

---

## 🌍 Map Server Setup (Docker)

1. Download map data (e.g., Portugal):
   ```bash
   wget 'https://download.geofabrik.de/europe/portugal-latest.osm.pbf'
   wget 'https://download.geofabrik.de/europe/portugal.poly'
   ```

2. Create Docker volumes and import data:
   ```bash
   docker volume create osm-data
   docker volume create osm-tiles
   docker run -e UPDATES=enabled \
      -v /absolute/path/to/portugal-latest.osm.pbf:/data/region.osm.pbf \
      -v /absolute/path/to/portugal.poly:/data/region.poly \
      -v osm-data:/data/database/ \
      overv/openstreetmap-tile-server import
   ```

3. Start the tile server:
   ```bash
   docker run -p 8080:80 \
      -v osm-data:/data/database/ \
      -v osm-tiles:/data/tiles/ \
      -d overv/openstreetmap-tile-server run
   ```

**Tile URLs** (Max Zoom: 19):  
- `http://localhost:8080/wmts/gm_layer/gm_grid/{level}/{x}/{y}.png`  
- `http://127.0.0.1:8080/tile/{level}/{x}/{y}.png`

---

## 📊 Metrics & Analysis

### 1. Extract Metrics
```bash
rosrun metrics_extractor metrics.py
```

### 2. Generate Plots
```bash
# Navigate to /logfiles first
python3 metrics_analyser_deep.py logfile_metrics-2025-02-03-14-53.csv -o ../plots
```

### 3. Create LaTeX Tables
```bash
python3 table_generator.py
```
