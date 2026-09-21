# Draw Letter D with UR3

ROS 2 Humble demo for simulating a UR3 robot drawing the Vietnamese letter `D`/`Đ` on a vertical plane using Gazebo, MoveIt 2 and RViz.

## Requirements

- Ubuntu with ROS 2 Humble
- Gazebo / Ignition Fortress packages used by Universal Robots ROS 2 simulation
- MoveIt 2
- Universal Robots ROS 2 packages:
  - `ur_description`
  - `ur_moveit_config`
  - `ur_simulation_gz`

## Build

Clone this repository into a ROS 2 workspace, or use it as the workspace root:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ur3_d
source install/setup.bash
```

## Run The Vertical Letter D Demo

Launch Gazebo, MoveIt, RViz and the writer node:

```bash
ros2 launch ur3_d draw_letter_d_red.launch.py
```

The demo waits until Gazebo, controllers, joint states and TF are ready, then automatically starts drawing.

In RViz:

- Red marker: planned letter path.
- Blue marker: actual ink trace from the end effector.
- The drawing plane is vertical, using the `xz` plane.

## Headless Test

To run without Gazebo GUI and RViz:

```bash
ros2 launch ur3_d draw_letter_d_red.launch.py gazebo_gui:=false launch_rviz:=false
```

## Useful Files

- `src/TuongTacNguoi_Robot/ur3_d/launch/draw_letter_d_red.launch.py`: main launch file for this demo.
- `src/TuongTacNguoi_Robot/ur3_d/config/writer_vertical_red.yaml`: drawing plane, marker colors and writer settings.
- `src/TuongTacNguoi_Robot/ur3_d/src/write_letter_d_node.cpp`: writer node.
- `src/TuongTacNguoi_Robot/ur3_d/src/letter_d_path.cpp`: creates the letter waypoints.
- `src/TuongTacNguoi_Robot/ur3_d/src/moveit_executor.cpp`: plans and executes MoveIt trajectories.

## Troubleshooting

If RViz shows red RobotModel errors or the launch reports existing simulation nodes, stop old terminals with `Ctrl+C` before launching again.

If using a custom ROS domain, use the same value in every terminal:

```bash
export ROS_DOMAIN_ID=10
```
