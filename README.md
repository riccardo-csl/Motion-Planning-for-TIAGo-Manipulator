# Motion Planning for TIAGo Manipulator

A compact workspace showcasing end-to-end motion planning for the PAL Robotics TIAGo manipulator in simulation. It integrates ArUco-based perception, frame transformations, head scanning, inverse kinematics (IK) planning, and a state-machine driven pick & place. A second task extends the pipeline with Learning by Demonstration (LbD) using Dynamic Movement Primitives (DMP) to generalize demonstrated trajectories.

## Tasks

- Task 2 (folder `T2_G_7`)
  - Detect ArUco markers from the head camera, transform poses to `base_footprint`.
  - Head sweep for table scanning, IK-based arm planning, and a state machine for pick & place.
  - Launch: `ros2 launch my_bringup action_app.launch.py`
- Task 3 (folder `T3_G_7`)
  - Adds LbD via Cartesian DMPs to learn from demos and adapt to new goals.
  - Automatic selection of nearest pick/place points based on detected markers.
  - Launch: `ros2 launch my_bringup action_app_2.launch.py`

See detailed docs and video previews in:
- `T2_G_7/README.md`
- `T3_G_7/README.md`

## Repository Layout

- `T2_G_7/`
  - `head_scan/` (ROS 2 Python nodes)
  - `my_bringup/` (ROS 2 launch package)
  - `urdf_config.rviz`, `video_task_2.mp4`
- `T3_G_7/`
  - `head_scan/` (adds `lbd_motion_planner.py`, `lbd_state_machine.py`, demo files)
  - `my_bringup/` (LbD launch)
  - `video_task_3.mp4`

## Requirements (summary)

- ROS 2 (tested in simulation), Gazebo, RViz2, TF2.
- Python: `numpy`, `scipy`, `opencv-contrib-python`, `roboticstoolbox-python`, `spatialmath-python`.
- For LbD (Task 3): `movement-primitives`, `pytransform3d`.

Install example (with your ROS 2 environment sourced):

```bash
pip install numpy scipy opencv-contrib-python roboticstoolbox-python spatialmath-python \
            movement-primitives pytransform3d
```

## Quick Start

1. Create a ROS 2 workspace and copy the desired task packages:
   - Task 2: copy `T2_G_7/head_scan` and `T2_G_7/my_bringup` into `src/`
   - Task 3: copy `T3_G_7/head_scan` and `T3_G_7/my_bringup` into `src/`
2. Build and source:

```bash
colcon build --packages-select head_scan my_bringup
source install/setup.bash
```

3. Launch:

```bash
# Task 2
ros2 launch my_bringup action_app.launch.py
# Task 3 (LbD)
ros2 launch my_bringup action_app_2.launch.py
```

Notes
- RViz config is auto-detected by the launch files if present in the repo.
- URDF and demo paths are resolved relative to the package (no hard-coded absolute paths).
