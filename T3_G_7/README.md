# Task 3 — Learning by Demonstration (DMP) for TIAGo Pick & Place

This extends Task 2 by introducing a Learning by Demonstration (LbD) planner using Dynamic Movement Primitives (DMP) to generalize pick & place trajectories from recorded demonstrations.

## Video Preview

<p align="center">
  <a href="https://drive.google.com/file/d/1gpC3hEnnw7B0CAyb2zc3661rlNmfRyCc/view?usp=sharing" target="_blank">
    <img src="https://drive.google.com/thumbnail?id=FILE_ID_TASK_3&sz=w1000" alt="Task 3 video preview" width="800" />
  </a>
</p>

Alternatively: direct player link: https://drive.google.com/file/d/1gpC3hEnnw7B0CAyb2zc3661rlNmfRyCc/view?usp=sharing

## What’s New vs Task 2

- `lbd_motion_planner.py`: uses Cartesian DMPs (`movement_primitives`) to learn trajectories from demo files and adapt to new start/goal
- `lbd_state_machine.py`: automatically selects the nearest pick/place position based on detected ArUco markers and updates the execution sequence
- Demo folder: `head_scan/head_scan/tiago_traiettorie_SG/` with text files `pick_?_repN.txt`, `place_?_repN.txt` grouped by position pairs (e.g., `2a-4b`)
- Dedicated launch: `my_bringup/launch/action_app_2.launch.py` (starts the LbD nodes)

## Project Structure

- `head_scan`
  - `aruco_scan_publisher.py`: detects markers, publishes `/aruco_<id>_pose`
  - `aruco_coord_transformation.py`: transforms to `/aruco_<id>_pose_transformed`
  - `head_movement_client.py`: head sweep
  - `lbd_motion_planner.py`: DMP learning + generalization with IK (Robotics Toolbox)
  - `lbd_state_machine.py`: task sequence with automatic pick/place choice
  - `motion_planner_node.py`, `state_machine_node.py`: non‑LbD versions (kept for compatibility)
  - `tiago_robot.urdf`: robot URDF used for IK
  - `tiago_traiettorie_SG/`: demonstrations (pick positions `1..4` mapped to XY, and place positions `1..4`)
- `my_bringup`
  - `launch/action_app_2.launch.py`: time‑sequenced startup using LbD nodes

## LbD Execution Flow

1. ArUco detection/transformation and head sweep as in Task 2
2. `lbd_state_machine`
   - builds `final_pose` (pose+offsets+rotations)
   - picks the closest pick/place among predefined candidates
   - publishes `final_pose` and a command on `/sm_ik_communication` including: DMP number of steps and `pick_i`/`place_j`
3. `lbd_motion_planner`
   - loads the corresponding demo from files (time + joints) and resamples at constant step
   - converts to Cartesian trajectory (FK) and fits DMP weights
   - reconfigures the DMP with new start/goal and generates `Y_dmp_generalized`
   - solves IK per frame and publishes `/arm_controller/joint_trajectory` and `/torso_controller/joint_trajectory`
4. Synchronization: at the end of each segment, publishes `terminato` on `/sm_ik_communication` to advance the sequence

## Extra Requirements (beyond Task 2)

- Python: `movement_primitives`, `pytransform3d`

```bash
pip install movement-primitives pytransform3d
```

## Configuration (fix absolute paths)

Update hard‑coded references to local paths:

- `head_scan/head_scan/lbd_motion_planner.py`
  - `urdf_loc`: set the path to `tiago_robot.urdf` on your system
  - demo directory: replace `"/home/.../tiago_traiettorie_SG/"` with the actual path to the `tiago_traiettorie_SG` folder in the package
- `my_bringup/launch/action_app_2.launch.py`
  - RViz `-d`: point to a valid RViz config (e.g., `T2_G_7/urdf_config.rviz`)

Tip: use environment variables or launch parameters to make these paths configurable.

## Install & Build

1. Create a workspace and copy Task 3 packages:
   - `src/head_scan` from `T3_G_7/head_scan`
   - `src/my_bringup` from `T3_G_7/my_bringup`
2. Build and source the environment:

```bash
colcon build --packages-select head_scan my_bringup
source install/setup.bash
```

## Run

1. Start TIAGo simulation in Gazebo with controllers and camera active (as in Task 2)
2. Launch the LbD version:

```bash
ros2 launch my_bringup action_app_2.launch.py
```

## Notes & Tips

- Automatic pick/place selection: `lbd_state_machine.py` chooses among 4 candidate positions per area (pick/place) based on XY distance to ArUco markers
- IK fallback: if LbD metadata isn’t provided, the planner runs the Task‑2‑style interpolated IK
- IK robustness: `lbd_motion_planner` uses `ikine_LM` and falls back to the last valid configuration on local failures
- Initial posture consistency: the pre‑positioning sequence in launch (torso/arm) differs from Task 2 and matches the demo dataset
