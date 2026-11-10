# 🚀 ROS 2 Autonomous SLAM & Navigation with Andino AMR

<div align="center">

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-3498DB?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Gazebo Fortress](https://img.shields.io/badge/Gazebo-Fortress-FF6F00?style=for-the-badge&logo=gazebo&logoColor=white)](https://gazebosim.org/)
[![Docker](https://img.shields.io/badge/Docker-Containerized-2496ED?style=for-the-badge&logo=docker&logoColor=white)](https://www.docker.com/)
[![Python 3.10](https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)
[![SLAM Toolbox](https://img.shields.io/badge/SLAM-Toolbox-4CAF50?style=for-the-badge)](https://github.com/SteveMacenski/slam_toolbox)
[![Nav2](https://img.shields.io/badge/Nav2-Navigation2-E91E63?style=for-the-badge)](https://navigation.ros.org/)

<p align="center">
  <b>An end-to-end autonomous mobile robotics pipeline featuring 2D SLAM, intelligent obstacle-avoidance exploration, and Nav2 navigation in ROS 2 & Gazebo Fortress.</b>
</p>

<img src="images/autonomous_navigation.gif" alt="Autonomous Navigation Demo" width="900" style="border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.15);"/>

</div>

---

## ⚡ Quick Start: How to Run

To run the complete simulation, online SLAM, and autonomous mapping pipeline:

> **Prerequisite (Host):** Run `xhost +` once to grant display access to Docker:
> ```bash
> xhost +
> ```

### 1️⃣ Start the Docker Container
In your main terminal, start the Docker container:

```bash
docker compose up
```

---

### 2️⃣ Terminal 1: Launch Gazebo Simulation & Robot
Open a new terminal, enter the container, and launch the simulation:

```bash
docker exec -it robotics_essentials_ros2 bash
ros2 launch andino_gz andino_gz.launch.py
```

> 💡 *In Gazebo, press the **Play (▷)** button in the bottom-left toolbar to start physics!*

---

### 3️⃣ Terminal 2: Launch SLAM Toolbox
Open a second terminal, enter the container, and start the SLAM node:

```bash
docker exec -it robotics_essentials_ros2 bash
ros2 launch andino_gz slam_toolbox_online_async.launch.py
```

---

### 4️⃣ Terminal 3: Autonomous Mapping & Exploration
Open a third terminal, enter the container, and execute the autonomous explorer:

```bash
docker exec -it robotics_essentials_ros2 bash
auto_explore
```

The robot will:
1. Rotate $360^\circ$ on the spot to capture an initial panoramic scan.
2. Drive through the warehouse, dynamically sensing walls and obstacles.
3. Automatically bypass obstacles using smooth arc steering.
4. Construct the complete 2D map in RViz without any manual teleoperation!

---

### 5️⃣ Save the Generated Map
Once the warehouse is fully mapped, press `Ctrl + C` in Terminal 3 and save the map:

```bash
ros2 run nav2_map_server map_saver_cli --free 0.15 --fmt png -f $HOME/andino_map
```

---

### 6️⃣ Autonomous Nav2 Goal Navigation
Stop previous simulation nodes (`Ctrl + C`), then launch simulation with Nav2:

```bash
docker exec -it robotics_essentials_ros2 bash
ros2 launch andino_gz andino_gz.launch.py nav2:=True
```

- In RViz, click **2D Pose Estimate** to initialize the robot's pose on the map.
- Click **Nav2 Goal** anywhere on the map to command the robot to drive autonomously to the target!

---

## 🌟 Key Features

- 🤖 **Smart Autonomous Exploration (`auto_explore`)**: Self-driving wanderer node with real-time obstacle avoidance, dynamic corridor safety, and corner trap escape recovery.
- 📐 **Rigorous Sensor Frame Calibration**: Solves the physical RPLiDAR $180^\circ$ yaw mounting inversion using real-time Cartesian transforms into `base_link`.
- 🗺️ **State-of-the-Art 2D SLAM**: Powered by `slam_toolbox_online_async` with online pose-graph optimization and robust loop closure.
- 🎯 **Complete Nav2 Integration**: Global & local costmaps, DWB trajectory controller, AMCL particle filter localization, and behavior recovery trees.
- 🖥️ **Pre-Configured RViz2**: Custom layout displaying Map, LaserScan, TF frames, and costmap layers with no manual setup needed.
- 🐳 **100% Dockerized**: Zero dependency headaches. Everything runs in isolated Ubuntu 22.04 + ROS 2 Humble containers with hardware-accelerated GPU passthrough.

---

## 🏗️ System Architecture

```mermaid
flowchart TD
    subgraph Gazebo_Simulation ["Gazebo Fortress Simulation (Andino AMR)"]
        GZ_ROBOT["Andino Robot Model"]
        GZ_LIDAR["RPLiDAR A1 (720 samples, 12m)"]
        GZ_ODOM["Diff Drive Odometry"]
    end

    subgraph ROS_Bridge ["ROS-Gazebo Bridge (ros_gz_bridge)"]
        SCAN["/scan (sensor_msgs/LaserScan)"]
        ODOM["/odom (nav_msgs/Odometry)"]
        CMD["/cmd_vel (geometry_msgs/Twist)"]
    end

    subgraph Auto_Exploration ["Custom Smart Auto Explorer"]
        CALIB["RPLiDAR 180° Calibration"]
        CORRIDOR["Dynamic Safety Corridor (0.56m)"]
        BYPASS["Smooth Arc Bypass Engine"]
        ESCAPE["Trap & Corner Escape Recovery"]
    end

    subgraph Mapping_Stack ["SLAM & Navigation Stacks"]
        SLAM["slam_toolbox (Online Async SLAM)"]
        MAP_SERVER["nav2_map_server (Map Saver & Loader)"]
        NAV2["Nav2 Stack (AMCL + Costmaps + Planners)"]
    end

    subgraph Visualization ["RViz2 (Pre-configured Layout)"]
        RVIZ["RViz2 Visualization GUI"]
    end

    GZ_LIDAR --> SCAN
    GZ_ODOM --> ODOM
    SCAN --> CALIB
    CALIB --> CORRIDOR --> BYPASS --> CMD
    ESCAPE --> CMD
    CMD --> GZ_ROBOT

    SCAN --> SLAM
    ODOM --> SLAM
    SLAM -->|"/map"| RVIZ
    SLAM --> MAP_SERVER

    MAP_SERVER --> NAV2
    NAV2 --> CMD
    NAV2 --> RVIZ
```

---

## 🧮 Sensor Calibration & Mathematical Model

### The RPLiDAR $180^\circ$ Inversion Challenge
On physical differential drive robots (including Andino), LiDAR sensors are frequently mounted facing backward ($180^\circ$ yaw rotation) to route USB and power cables toward the robot's interior:

$$\text{Joint Transform: } \quad \text{rpy} = (0,\, 0,\, \pi)$$

When processing raw laser beams without coordinate transformation, angle $\theta = 0$ corresponds to the **rear** of the robot, causing the robot to mistakenly treat front walls as non-existent!

### Body Frame Cartesian Mapping
The `auto_explore` node resolves this by transforming all 720 beam samples directly into the robot's actual motion frame (`base_link`):

$$\begin{cases}
x_{\text{robot}} = -r \cdot \cos(\theta_{\text{laser}}) + 0.05 \\[4pt]
y_{\text{robot}} = -r \cdot \sin(\theta_{\text{laser}})
\end{cases}$$

Where:
- $x_{\text{robot}} > 0$: **Forward ahead**
- $y_{\text{robot}} > 0$: **Left flank**
- $y_{\text{robot}} < 0$: **Right flank**

### Dynamic Safety Corridor
An active safety corridor of width $W = 0.56\text{ m}$ ($|y_{\text{robot}}| < 0.28\text{ m}$) monitors the vehicle's clearance:
- **$x_{\text{robot}} < 0.45\text{ m}$ (Critical Zone)**: $v = 0$, immediate high-speed rotation on the spot.
- **$0.45\text{ m} \le x_{\text{robot}} < 0.95\text{ m}$ (Bypass Zone)**: Smooth speed deceleration coupled with dynamic arc steering:

$$v = v_{\text{min}} + \left(\frac{x_{\text{min}} - d_{\text{crit}}}{d_{\text{slow}} - d_{\text{crit}}}\right) \cdot (v_{\text{max}} - v_{\text{min}})$$

$$\omega = (1 - \text{speed\_ratio}) \cdot \omega_{\text{max}} \cdot \text{dir}_{\text{clearance}} + \omega_{\text{wall\_repulse}}$$

---

## ⚡ Alternative Shortcut Runners

If you prefer one-click host execution without manually attaching to Docker, you can also run the helper scripts directly from your host machine:

| Step | Script | Action |
|:---:|:---|:---|
| **1** | `./1_start_sim.sh` | Launches Gazebo world (`depot.sdf`) & pre-configured RViz2 |
| **2** | `./2_start_slam.sh` | Launches `slam_toolbox` online asynchronous mapping |
| **3** | `./3_auto_explore.sh` | Launches the autonomous obstacle-avoidance & exploration node |
| **4** | `./4_save_map.sh` | Saves the generated 2D occupancy map (`.yaml` + `.png`) |
| **5** | `./5_start_navigation.sh` | Starts the Nav2 autonomous goal-navigation stack |

---

## 📂 Project Directory Structure

```plaintext
Slam/
├── 0-setup/                   # Installation and Docker setup tutorials
├── 1-ros_2_introduction/      # Fundamental ROS 2 exercises (topics, tf)
├── 2-slam_and_navigation_demo/ # SLAM and Nav2 exercises
├── 3-create_ros_2_package/    # Custom package creation guide
├── 4-robot_odometry/          # Odometry calculation and wheel kinematics
├── 5-path_planning/           # Nav2 planner plugin development
├── docker/                    # Dockerfile, dependencies & docker-compose
│   ├── Dockerfile
│   ├── docker-compose.yaml
│   └── ros_entrypoint.sh
├── images/                    # Demo illustrations, diagrams, and gifs
├── packages/                  # ROS 2 packages (custom_nav2_planner, msgs)
├── auto_explore.py            # 🧠 Intelligent autonomous exploration node
├── 1_start_sim.sh             # 🚀 Runner: Gazebo & RViz2
├── 2_start_slam.sh            # 🗺️ Runner: Slam-Toolbox
├── 3_auto_explore.sh          # 🤖 Runner: Autonomous exploration
├── 3_teleop.sh                # ⌨️ Runner: Manual keyboard teleop (optional)
├── 4_save_map.sh              # 💾 Runner: Save 2D map
├── 5_start_navigation.sh      # 🧭 Runner: Nav2 autonomous navigation
├── docker-compose.yaml        # Root Docker Compose file
└── README.md                  # Project documentation
```

---

## 📡 Key ROS 2 Topics & Services

| Name | Type | Description |
|:---|:---|:---|
| `/scan` | `sensor_msgs/msg/LaserScan` | 2D LiDAR point measurements (720 samples, $360^\circ$) |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Differential drive velocity commands ($v_x, \omega_z$) |
| `/odom` | `nav_msgs/msg/Odometry` | Wheel encoder odometry ground truth |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 2D occupancy grid generated by SLAM |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Coordinate frame tree (`map` $\to$ `odom` $\to$ `base_link` $\to$ `rplidar_laser_link`) |
| `/map_server/load_map` | `nav2_msgs/srv/LoadMap` | Service to dynamically load custom maps into Nav2 |

---

## 🔧 Troubleshooting & Common Errors

<details>
<summary><b>1. "Authorization required, but no authorization protocol specified" or "libEGL warning: failed to open /dev/dri/renderD128: Permission denied"</b></summary>

This error occurs when the container cannot access the host's X11 display server or GPU rendering device.
**Fix**: Run `xhost +` in a host terminal before running the simulation:
```bash
xhost +
```
Additionally, `/dev/dri` and user video group permissions have already been configured in `docker-compose.yaml`.
</details>

<details>
<summary><b>2. Gazebo window freezes or robot does not move</b></summary>

Make sure physics simulation is active in Gazebo by pressing the **Play (▷)** button in the lower toolbar.
</details>

<details>
<summary><b>3. Custom Map Loading into Nav2</b></summary>

To load your newly saved map into the running Nav2 session without relaunching:
```bash
docker exec -it robotics_essentials_ros2 /ros_entrypoint.sh \
  ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /home/user/andino_map.yaml}"
```
</details>

---

## 👥 Credits & Acknowledgements

- **Maintained & Enhanced by**: [Alireza Akbari](https://github.com/Alirezakbari)
- **Course Originator**: Adapted from ROS 2 coursework at Arak University of Technology by [Dr. Kaveh Hooshmandi](https://github.com/kaveh-hooshmandi).
- **Original Authors**: Janne Karttunen, George-Cosmin Porusniuc, and Prof. Ilkka Jormanainen (University of Eastern Finland).
- **Robot Platform**: Andino AMR simulation developed by [Ekumen](https://github.com/Ekumen-OS).

---

<div align="center">
  <b>⭐ If you find this project helpful, please consider starring the repository! ⭐</b>
</div>
