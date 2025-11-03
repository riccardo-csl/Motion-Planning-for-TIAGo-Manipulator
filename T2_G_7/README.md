# Task 2 — ArUco Detection, Transformations and Pick & Place with TIAGo

This workspace implements a complete pipeline on TIAGo (simulation) to:

- detect ArUco markers from the head camera and estimate their pose in the camera frame
- transform those poses to the robot base frame (`base_footprint`)
- perform a head sweep to scan the table
- plan the arm motion (IK) and coordinate a pick & place task via a state machine

## Video Preview

<p align="center">
  <a href="https://drive.google.com/file/d/14Jrj5EGsz4LSt3l2j6b3D4twWaWzWBFn/view?usp=sharing" target="_blank">
    <img src="https://drive.google.com/thumbnail?id=14Jrj5EGsz4LSt3l2j6b3D4twWaWzWBFn" width="700"/>
  </a>
</p>

Alternatively: direct player link: https://drive.google.com/file/d/14Jrj5EGsz4LSt3l2j6b3D4twWaWzWBFn/view?usp=sharing

## Project Structure

- `head_scan`: ROS 2 (Python) package with pipeline nodes
  - `aruco_scan_publisher.py`: detects ArUco markers and publishes poses in the camera frame
  - `aruco_coord_transformation.py`: transforms ArUco poses to `base_footprint` and averages over 20 s
  - `head_movement_client.py`: head sweep via Action Client
  - `motion_planner_node.py`: arm planning via IK (Robotics Toolbox), publishes trajectories to Gazebo
  - `state_machine_node.py`: state machine orchestrating pick & place (gripper/torso/IK) from ArUco poses
  - `tiago_robot.urdf`: URDF used by the planner for IK
- `my_bringup`: ROS 2 (CMake) package with the main launch
  - `launch/action_app.launch.py`: time-sequenced startup of nodes and RViz
- `urdf_config.rviz`: RViz config to visualize frames and transformed poses
- `screenshot_configurazione_rviz_task_2.png`: RViz configuration snapshot

## Execution Flow

1. ArUco scan: `aruco_scan_publisher` subscribes to `head_front_camera/rgb/image_raw` and publishes
   - IDs to `aruco_poses` (`std_msgs/Int32`)
   - poses to `aruco_<id>_pose` (`geometry_msgs/PoseStamped`)
2. Transformation: `aruco_coord_transformation` listens to TF camera→base, transforms and publishes a 20 s average
   - poses to `aruco_<id>_pose_transformed` (`geometry_msgs/PoseStamped`)
3. Head sweep: `head_movement_client` moves the head between two predefined poses
4. Pre‑positioning: timed commands for torso and arm via `ros2 topic pub` (starting posture)
5. Planning: `motion_planner_node` computes IK trajectories from the current state to the `final_pose`
6. State machine: `state_machine_node` decodes a list of tasks (poses+offsets+rotations, gripper, torso, relax) and coordinates IK

## Core Nodes and Topics

- `head_scan/aruco_scan_publisher`
  - sub: `/head_front_camera/rgb/image_raw` (`sensor_msgs/Image`), `/head_front_camera/rgb/camera_info` (`sensor_msgs/CameraInfo`)
  - pub: `/aruco_poses` (`std_msgs/Int32`), `/aruco_<id>_pose` (`geometry_msgs/PoseStamped`)
- `head_scan/aruco_coord_transformation`
  - sub: `/aruco_poses`, `/aruco_<id>_pose`
  - TF: `head_front_camera_optical_frame` → `base_footprint`
  - pub: `/aruco_<id>_pose_transformed` (20 s average)
- `head_scan/motion_planner_node`
  - sub: `/final_pose` (`geometry_msgs/PoseStamped`), `/arm_controller/joint_trajectory`, `/torso_controller/joint_trajectory` (to track current pose)
  - pub: `/arm_controller/joint_trajectory`, `/torso_controller/joint_trajectory`, sync on `/sm_ik_communication` (`std_msgs/String`)
- `head_scan/state_machine_node`
  - sub: `/aruco_1_pose_transformed`…`/aruco_4_pose_transformed`, `/sm_ik_communication`
  - pub: `/final_pose`, `/gripper_controller/joint_trajectory`, `/torso_controller/joint_trajectory`, Gazebo attach/detach service calls

## Requirements

- ROS 2 (tested in simulation with TIAGo, Gazebo)
- Python packages: `opencv-contrib-python`, `numpy`, `scipy`, `cv_bridge`, `roboticstoolbox-python`, `spatialmath-python`
- RViz2, TF2, Gazebo and `gazebo_ros_link_attacher` (for object attach/detach)

Install example (ROS 2 environment active):

```bash
pip install opencv-contrib-python numpy scipy roboticstoolbox-python spatialmath-python
```

## Installation

1. Create a workspace and copy Task 2 packages:
   - `src/head_scan` from `T2_G_7/head_scan`
   - `src/my_bringup` from `T2_G_7/my_bringup`
2. Build:

```bash
colcon build --packages-select head_scan my_bringup
source install/setup.bash
```

## Configuration (fix absolute paths)

Some files may contain absolute paths from the original development environment. Update them before running:

- `head_scan/head_scan/motion_planner_node.py`: `urdf_loc` to your local `tiago_robot.urdf`
- `my_bringup/launch/action_app.launch.py`: RViz `-d` path. Point to `T2_G_7/urdf_config.rviz` or your absolute path

RViz example: `-d /absolute/path/to/repo/T2_G_7/urdf_config.rviz`

## Run

1. Start TIAGo simulation in Gazebo with controllers enabled:
   - `/head_controller/follow_joint_trajectory`
   - `/arm_controller/joint_trajectory`, `/torso_controller/joint_trajectory`, `/gripper_controller/joint_trajectory`
   - head camera: `/head_front_camera/rgb/image_raw` and `/head_front_camera/rgb/camera_info`
2. Launch Task 2 pipeline:

```bash
ros2 launch my_bringup action_app.launch.py
```

At startup, after the head sweep and pre‑positioning, the planner and state machine will run the pick & place sequence.

## Tips

- RViz: configuration snapshot — `screenshot_configurazione_rviz_task_2.png`
- Attach/Detach: the state machine uses `gazebo_ros_link_attacher` to attach/detach named objects (e.g., `cocacola`, `pringles`)
- ArUco averaged topics: each marker is published once as the average over 20 s after the first detection
