# Uncertainty-Aware MPC: Probabilistic SDFs & Active Visibility

![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-3499cd.svg)\
![Status](https://img.shields.io/badge/Status-Master's_Thesis-orange.svg)\
![Build](https://img.shields.io/badge/Build-Colcon-blue.svg)

This repository contains the implementation for the Master's Thesis:\
***"Uncertainty-Aware MPC: Probabilistic SDFs and Active Visibility for
Dynamic Clutter"***.

The framework integrates a probabilistic environment model (**Voxblox
ROS 2 migration**) with a **Model Predictive Control (MPC)** planner.\
It enables a **Franka Emika Panda** robot to navigate **dynamic,
cluttered environments** while maintaining a probabilistic world model
and actively maximizing target visibility ("peeking").

## 📌 Project Overview

This system addresses motion-planning in **uncertain, dynamic
environments**, where both collision avoidance and visual observability
must be optimized.

### Key Features

-   **Probabilistic Environment Mapping**
    -   ROS 2-migrated **Voxblox** for TSDF, ESDF, and Occupancy.
    -   Supports real-time incremental updates.
-   **Uncertainty Handling**
    -   Bayesian update for voxel probabilities.
    -   Memory decay for dynamic obstacles (e.g., other robots).
-   **Active MPC Planning**
    -   MPC cost reformulated using **ESDF gradients**.
    -   Real-time differentiable **ray-casting** for maximizing
        visibility to targets.

## 🏗️ System Architecture

1.  **`tsdf_server_node`**\
    Consumes depth/pointcloud data to build a Truncated Signed Distance
    Field (TSDF).

2.  **`esdf_server_node`**\
    Computes the Euclidean Signed Distance Field (ESDF), used by the MPC
    to compute collision gradients.

3.  **MPC Planner (External Node)**\
    Subscribes to ESDF, computes optimal control.

## ⚙️ Installation

### Prerequisites

-   Ubuntu 22.04
-   ROS 2 Humble
-   colcon, rosdep

## 🔧 Build Instructions

``` bash
mkdir -p ~/ws_thesis/src
cd ~/ws_thesis/src
git clone <YOUR_REPO_URL>

cd ~/ws_thesis
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

# 🚀 Usage

## Running Dataset Experiments

``` bash
ros2 launch voxblox_ros cow_and_lady_dataset.launch.py
```

## Key Parameters

  Parameter                  Value      Reason
  -------------------------- ---------- -----------------
  generate_esdf              True       Required
  update_esdf_every_n\_sec   0.5--1.0   High frequency
  update_mesh_every_n\_sec   0.0        Disabled
  publish_pointclouds        False      Reduce traffic
  publish_slices             True       Debug
  tsdf_voxel_size            0.05 m     Good resolution

# 📊 Visualization

Use RViz displays: - **ESDF PointCloud:**
`/voxblox_node/esdf_pointcloud` - **ESDF Slice:**
`/voxblox_node/esdf_slice`

# 🛠️ Migration Status

  Component         Status
  ----------------- -------------
  TSDF Server       ✅
  ESDF Server       ✅
  Mesh Integrator   🚧 Disabled
  Evaluator Nodes   ❌

# 👨‍🎓 Thesis Information

Author: Joel Agustin Sanchez\
Supervisors: Prof. Dr.-Ing. Steven Liu, M.Sc. Chen Cai\
Institution: RPTU Kaiserslautern-Landau

# 📄 Citation

``` bibtex
@inproceedings{oleynikova2017voxblox,
  author={Oleynikova, Helen and Taylor, Zachary and Fehr, Marius and Siegwart, Roland and Nieto, Juan},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  title={Voxblox: Incremental 3D Euclidean Signed Distance Fields for On-Board MAV Planning},
  year={2017}
}
```
