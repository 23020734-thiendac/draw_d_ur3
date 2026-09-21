# Mo phong UR3 ve chu D tren mat phang dung

Day la demo ROS 2 Humble mo phong robot UR3 ve chu `D`/`Đ` tren mat phang dung bang Gazebo, MoveIt 2 va RViz.

## Yeu cau

- Ubuntu va ROS 2 Humble
- MoveIt 2
- Gazebo / Ignition Fortress
- Cac package UR ROS 2:
  - `ur_description`
  - `ur_moveit_config`
  - `ur_simulation_gz`

## Bien dich

Chay cac lenh sau trong workspace:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ur3_d
source install/setup.bash
```

## Chay chuong trinh ve chu D

Chay launch chinh:

```bash
ros2 launch ur3_d draw_letter_d_red.launch.py
```

Launch nay se khoi dong Gazebo, MoveIt, RViz va node ve chu. Sau khi robot, controller, joint states va TF san sang, chuong trinh tu dong bat dau ve.

Trong RViz:

- Duong mau do la quy dao du kien.
- Duong mau xanh nuoc bien la net muc thuc te cua dau cong tac.
- Chu duoc ve tren mat phang dung `xz`.

## Cac file chinh

- `src/TuongTacNguoi_Robot/ur3_d/launch/draw_letter_d_red.launch.py`: file launch de chay demo ve chu D.
- `src/TuongTacNguoi_Robot/ur3_d/config/writer_vertical_red.yaml`: cau hinh mat phang ve, mau marker va thong so ve.
- `src/TuongTacNguoi_Robot/ur3_d/src/write_letter_d_node.cpp`: node dieu khien qua trinh ve.
- `src/TuongTacNguoi_Robot/ur3_d/src/letter_d_path.cpp`: tao cac waypoint cua chu D.
- `src/TuongTacNguoi_Robot/ur3_d/src/moveit_executor.cpp`: lap ke hoach va thuc thi trajectory bang MoveIt.
