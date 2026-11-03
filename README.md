# Drone ArUco Precision Landing

A ROS 2 and Gazebo simulation of a drone searching for an ArUco marker and descending over its estimated position. The drone uses a downward-facing camera for marker detection and odometry for navigation.


## Demo

[▶ Watch the drone landing demonstration](docs/drone_aruco_landing.mp4)

The video demonstrates autonomous zigzag search, ArUco detection,
3D marker localization, and precision landing on marker ID 3.

## Project structure

```text
.
├── README.md
├── .gitignore
├── docs/
│   └── demo.mp4
└── ros_ws/
    └── src/
        ├── landing_control/
        │   ├── landing_control/
        │   │   ├── mission_node.py   # ROS interfaces and mission control
        │   │   ├── aruco.py          # Detection, pose estimation, and CSV evaluation
        │   │   └── geometry.py       # Coordinate transforms and zigzag waypoints
        │   ├── config/ground_truth.yaml
        │   └── launch/run.launch.py
        └── landing_description/
            ├── urdf/drone.xacro
            ├── worlds/arena.sdf
            ├── models/              # Drone meshes and ArUco models/textures
            ├── config/ros_gz_bridge.yaml
            └── launch/sim.launch.py
```

## Requirements

The workspace uses ROS 2 Jazzy and Gazebo Sim (`gz sim`). A desktop session is needed for the default launch, which opens Gazebo and RViz.

Required components include:

- ROS 2, `colcon`, and the ROS launch tools.
- `ros_gz_sim`, `ros_gz_bridge`, `robot_state_publisher`, `xacro`, and `rviz2`.
- `rclpy`, `cv_bridge`, and the standard ROS message packages.
- Python NumPy, PyYAML, and OpenCV with `cv2.aruco` support.

Install these components for your ROS distribution before building. The package manifests currently do not list every runtime dependency, so `rosdep` alone may not install everything.

## Build

Run from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
cd ros_ws
colcon build --symlink-install
source install/setup.bash
```

Source both setup files in each new terminal before running ROS commands. Rebuild after changing launch files, packaged configuration, or package structure.

## Run

From the built and sourced workspace:

```bash
ros2 launch landing_control run.launch.py
```

This starts the arena, spawns the drone, starts the ROS–Gazebo bridge and RViz, and starts the mission node after a seven-second delay. The default landing target is marker **3**.

To launch only the simulation:

```bash
ros2 launch landing_description sim.launch.py
```

Then, in another sourced terminal, start the mission separately:

```bash
ros2 run landing_control mission_node --ros-args \
  -p ground_truth_yaml:="$(ros2 pkg prefix --share landing_control)/config/ground_truth.yaml" \
  -p target_id:=3 \
  -p hover_altitude:=2.0 \
  -p land_speed:=0.10
```

Use either the combined launch or the separate mission command; running both creates two controllers publishing velocity commands.

## Mission logic

```text
WAITING → TAKEOFF → SEARCH → ALIGN → DESCEND → FINAL_DESCENT → LANDED
```

- **WAITING:** Wait for camera calibration and odometry.
- **TAKEOFF:** Keep the initial XY position and climb to the configured Z coordinate.
- **SEARCH:** Follow alternating rows of zigzag waypoints. The image callback switches to alignment after enough consecutive observations of the target marker. Completing the path without acquiring the target ends in `DONE`.
- **ALIGN:** Move above the estimated marker position while holding the search altitude.
- **DESCEND:** Track the marker and descend. Pause vertical motion when lateral error is too large.
- **FINAL_DESCENT:** Hold the locked XY position using odometry, allowing descent to continue when the marker no longer fits in the image.
- **LANDED:** Send zero velocity after reaching the configured stopping height.

In visual alignment or descent, an outdated target estimate causes `TARGET_LOST` and zero velocity. A fresh target returns the controller to `ALIGN`. Excessive XY drift during final descent also causes `TARGET_LOST`.

The control loop runs at 20 Hz. Linear velocity commands are sent in the Gazebo world frame. `hover_altitude` and `land_z` are Z coordinates in the odometry frame, not heights measured relative to the takeoff point.

## Configuration

Mission parameters are declared in `mission_node.py`. They can be overridden at startup with `ros2 run ... --ros-args -p name:=value`, as shown above. The combined launch currently sets the target ID and marker configuration path directly in `run.launch.py`; it does not expose mission parameters as launch arguments.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `target_id` | `3` | Marker selected for landing |
| `min_pose_observations` | `3` | Consecutive target observations before alignment |
| `max_reprojection_px` | `5.0` | Maximum accepted reprojection error in pixels |
| `max_detection_age` | `0.35` | Maximum target estimate age in seconds |
| `hover_altitude` | `2.0` | Search Z coordinate in meters |
| `xmin`, `xmax` | `-3.8`, `3.8` | Search X bounds in meters |
| `ymin`, `ymax` | `-2.6`, `2.6` | Search Y bounds in meters |
| `spacing` | `1.5` | Search row spacing in meters |
| `wp_tolerance` | `0.1` | 3D waypoint arrival tolerance in meters |
| `align_tolerance` | `0.08` | XY tolerance for initial descent and final lock |
| `land_speed` | `0.10` | Descent speed in meters per second, shared by both phases |
| `land_z` | `0.18` | Z coordinate at which descent stops |
| `final_start_z` | `0.48` | Maximum Z coordinate for final lock |
| `final_stable_ticks` | `8` | Consecutive control cycles required for final lock |
| `max_final_xy_drift` | `0.15` | Maximum deviation from locked XY in meters |
| `camera_xyz` | `[0.0, 0.0, -0.015]` | Camera mounting translation in meters |
| `camera_rpy` | `[0.0, 1.5708, 0.0]` | Camera mounting rotation in radians |
| `output_csv` | `~/aruco_measurements.csv` | Measurement output path |

The descent pause threshold is currently fixed at **0.12 m** in `control()`. Parameter values are copied at initialization; changing ROS parameters while the node runs does not update that copy.

### Marker configuration

`ros_ws/src/landing_control/config/ground_truth.yaml` contains marker sizes and reference positions:

```yaml
markers:
  3:
    size: 0.30
    ground_truth: [-2.50, 2.70, 0.07]
```

Sizes and positions are in meters. Marker sizes feed pose estimation; reference positions are used only to evaluate the resulting estimates. They are not used as navigation targets. Markers without configured sizes are skipped. Keep sizes consistent with the marker models and camera mounting parameters consistent with `drone.xacro`.

## ROS topics

| Topic | Direction relative to mission node | Content |
| --- | --- | --- |
| `/camera/image_raw` | Input | Camera images |
| `/camera/camera_info` | Input | Camera intrinsics and distortion coefficients |
| `/odom` | Input | Drone position and orientation |
| `/cmd_vel` | Output | World-frame linear velocity |
| `/landing/state` | Output | Current mission state |
| `/aruco/position_camera` | Output | Marker position in the assumed optical frame |
| `/aruco/position_odom` | Output | Marker position in the odometry frame |

Position topics publish individual detections without a marker ID field. With several markers visible, messages for different markers share these topics.

## Measurement output

The node writes CSV measurements to `~/aruco_measurements.csv` by default. Each run overwrites the selected file; use a different `output_csv` to preserve earlier runs.

Recorded fields include time, marker ID, camera position and range, odometry-frame position, reference position, absolute errors per axis, 3D position error, and reprojection error. There is no percentage error field.

The 3D position error is:

```text
sqrt((estimated_x - reference_x)^2
   + (estimated_y - reference_y)^2
   + (estimated_z - reference_z)^2)
```

These measurements evaluate marker localization, not the final drone-to-marker landing offset.

## Current limitations

- `LANDED` means the stopping Z coordinate was reached; physical ground contact is not checked.
- Images use the latest available odometry rather than time-synchronized odometry. Incoming odometry has no freshness timeout.
- Target freshness uses image processing time rather than the image timestamp.
- The mission launch does not currently enable simulation time or bridge `/clock`, so timing uses the system clock.
- Camera optical axes and odometry/world alignment are assumed by the coordinate transform and should be checked when changing the robot or simulation setup.
