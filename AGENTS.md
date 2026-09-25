# Project verification

- Build the robot and run the control, dynamic_tracker, planner, and map_memory test suites with `./watod build robot`. The Dockerfile uses a non-default colcon install base; test commands must use the same `--install-base`.
- Changes to the Gazebo world or bridge require `./watod build gazeboserver`.
- Prefer Linux Docker tests over native macOS ROS tests: the native Conda environment can compile but has encountered libatomic/Python dynamic-linking failures.
- `/odom/filtered` describes the lidar pose, not the drive axle. In the current SDF the lidar is 1.3 m ahead of the axle. The chassis spans axle-relative x [-0.5, 1.5] m; the wheels extend the half-width to 0.7 m.
- Lidar and TF stamps use simulation time. Use scan-time transforms for mapping/tracking and timestamped predictions for collision checks; do not calculate obstacle velocity with wall-clock deltas.
- `/simulation_poses` is simulator ground truth for obstacle actuation and evaluation only. The navigation tracker must continue to use lidar observations, not ground truth.
