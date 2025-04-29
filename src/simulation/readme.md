# 🛡️ Air Defense Simulation - ROS2/Gazebo Workspace

A complete air defense simulation built using **ROS 2 Humble** and **Gazebo** on **Ubuntu 22.04**.  
This project simulates dynamic aerial threats (balloons) and supports object spawning, motion, and perception using ROS 2 nodes and Gazebo plugins.

---

## 📂 Workspace Structure

```
ros2_ws/
├── src/
│   └── air_defense_sim/
│       ├── air_defense_sim/              # Python nodes
│       │   ├── balloon_mover.py          # Moves spawned balloons in the simulation
│       │   ├── camera_subscriber.py      # Processes camera feed for balloon detection
│       │   └── spawn_multiple_balloons.py# Spawns multiple balloon models dynamically
│       ├── launch/
│       │   └── sim.launch.py            # Launch file for starting the simulation
│       ├── models/
│       │   └── balloon/                 # Gazebo model definition for balloons
│       ├── worlds/
│       │   └── air_defense_world.sdf    # Custom Gazebo world file
│       └── setup.py                     # Python package setup
```

---

## 📋 System Requirements

- ***Ubuntu 22.04***
- **ROS 2 Humble**
- **Gazebo**
- **Python 3.10**

---

## 🔧 Installation

#### 1. Update and upgrade system
`sudo apt update && sudo apt upgrade -y`

#### 2. Install ROS2 Humble Desktop
`sudo apt install ros-humble-desktop -y`

#### 3. Install Gazebo ROS packages
`sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros ros-humble-gazebo-plugins ros-humble-gazebo-dev -y`

#### 4. Install Python and setuptools
`sudo apt install python3-pip -y`
`pip install --upgrade setuptools`

## 🔄 ROS2 and Workspace Environment Setup
To avoid sourcing ROS2 and the workspace manually every time, add the following lines to your ~/.bashrc:

#### Source ROS2 setup
`source /opt/ros/humble/setup.bash`

#### Source your workspace after building
`source ~/ros2_ws/install/setup.bash`

#### Apply the changes:

`source ~/.bashrc`

## ⚙️ Building the Workspace
### Follow these steps:

#### Navigate to workspace root
`cd ~/ros2_ws`

#### Build the workspace
`colcon build`

#### Source the install space (if not already done via .bashrc)
`source install/setup.bash`

## 🚀 Launching the Simulation
### Start the simulation using:

`ros2 launch air_defense_sim sim.launch.py`

This will:
- Load the custom Gazebo world (air_defense_world.sdf)
- Spawn balloon models
- Start ROS2 nodes such as balloon_mover and camera_subscriber

## 🧩 Node Overview

| Node Name                   | Description                                                                 |
|-----------------------------|-----------------------------------------------------------------------------|
| `spawn_multiple_balloons.py` | Spawns multiple balloon models randomly in Gazebo                          |
| `balloon_mover.py`           | Moves balloons across the world to simulate threats                        |
| `camera_subscriber.py`       | Subscribes to the camera topic and processes images                        |

## 🗺️ Models and World

| Component                   | Description                                                                 |
|-----------------------------|-----------------------------------------------------------------------------|
| `worlds/air_defense_world.sdf` | Defines the terrain, environment, and layout                             |
| `models/balloon/`            | Defines the visual and collision properties of the balloon                 |




## ✨ Final Notes
After cloning the repository, always build and source your workspace before running the simulation!

### ✅ Quick Start

```cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch air_defense_sim sim.launch.py
```
