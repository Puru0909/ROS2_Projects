# Project 1 — Launch Runbook (Day 4: Localization + Navigation)

Keep this open every session from here on — it reflects everything actually fixed and working so far.

---

## 0. Before every session — clean slate check

Run this first, every time, even if you think everything's closed:
```bash
ps aux | grep -E "ign gazebo|gz sim|slam_toolbox|amcl|ros2 launch|rviz2|teleop|parameter_bridge" | grep -v grep
```
If **anything** is listed from a previous session, kill it all:
```bash
pkill -9 -f "ign gazebo"
pkill -9 -f slam_toolbox
pkill -9 -f amcl
pkill -9 -f spawn_robot.launch
pkill -9 -f localization.launch
pkill -9 -f navigation.launch
pkill -9 -f rviz2
pkill -9 -f teleop_twist_keyboard
pkill -9 -f parameter_bridge
```
Re-run the `ps aux` line — it must come back **completely empty** before you launch anything.

**Critical rule, learned the hard way today: `slam_toolbox` and `amcl` must never run at the same time.** Both publish `map → odom`; running both together is what caused the flickering. `slam_toolbox` is for Day 3 (building a map). `localization.launch.py` (AMCL) is for Day 4 onward (using a map already built). Never both.

---

## 1. Terminal 1 — Robot + Simulation
```bash
cd ~/Downloads/ROS2_Projects/slam_nav_ws
source install/setup.bash
ros2 launch my_robot_description spawn_robot.launch.py
```
Click into the Gazebo window and press ▶ if it looks paused. Wait ~10 seconds before starting Terminal 2.

## 2. Terminal 2 — Localization (map_server + AMCL)
```bash
source ~/Downloads/ROS2_Projects/slam_nav_ws/install/setup.bash
ros2 launch my_robot_description localization.launch.py
```
Wait ~10 seconds before starting Terminal 3.

## 3. Terminal 3 — Navigation (planner, controller, costmaps, behavior tree)
```bash
source ~/Downloads/ROS2_Projects/slam_nav_ws/install/setup.bash
ros2 launch my_robot_description navigation.launch.py
```
Watch for a line indicating managed nodes are active, with **no `[FATAL]`** anywhere above it.

## 4. Terminal 4 — RViz2
```bash
source ~/Downloads/ROS2_Projects/slam_nav_ws/install/setup.bash
rviz2
```
In RViz2:
- **Fixed Frame** → `map`
- **Add → By topic** → `/map` → Map
- **Add → By topic** → `/scan` → LaserScan
- **Add → By display type** → RobotModel (Description Topic: `/robot_description`)
- **Add → By display type** → PoseArray → set Topic to `/particlecloud`

## 5. Terminal 5 — Teleop (only needed to help AMCL converge)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
**Close this (`Ctrl+C`) once localized** — leaving it open risks it fighting Nav2 for control of `/cmd_vel`.

---

## In RViz2: localize, then navigate

1. Click **2D Pose Estimate**, click-drag at the robot's real position and facing (check the Gazebo window to confirm heading).
2. Drive a full loop with teleop, **specifically past the obstacle box** — it's the one feature that breaks our nearly-square room's symmetry and lets AMCL converge on the *correct* pose instead of a plausible wrong one.
3. Confirm: the LaserScan traces the map's walls as one clean line, and the particle cloud is tight.
4. **Close the teleop terminal.**
5. Click **2D Goal Pose**, click-drag anywhere on open floor.
6. Watch for a green planned path, then the robot driving there with no further input.

---

## Known bugs already found and fixed (reference only — shouldn't recur)

| Symptom | Cause | Fix |
|---|---|---|
| Gazebo window opens then closes instantly | Qt/Wayland conflict on GNOME | `QT_QPA_PLATFORM=xcb` (already permanent via `.bashrc`) |
| RViz2 "Frame [map] does not exist" | Missing `odom→base_link` bridge | Custom `odom_to_tf.py` broadcaster node |
| `slam_toolbox` drops every scan | Wrong lidar `frame_id` from Gazebo | `<ignition_frame_id>lidar_link</ignition_frame_id>` in sensor block |
| `/joint_states` never publishes | `Model→JointState` bridge unsupported on Humble | `joint_state_publisher` ROS2 package instead |
| `planner_server` FATAL crash | Wrong plugin name format | `nav2_navfn_planner/NavfnPlanner` (`/` not `::`) |
| `behavior_server` FATAL crash | Same issue, different plugins | `nav2_behaviors/Spin`, `/BackUp`, `/DriveOnHeading`, `/Wait` |
| `bt_navigator` can't open XML | Bare filename, no path | Removed the two `default_nav_*_bt_xml` lines — use built-in defaults |
| Robot flickering/jumping in RViz2 | `slam_toolbox` + `amcl` running simultaneously | Kill `slam_toolbox` before any Day 4 session |

---

## Where we are right now

Rotation-to-goal works (proves the full control chain is wired correctly). Forward motion after rotating does not yet — currently diagnosing whether the controller is generating zero forward velocity, or generating it but losing it downstream at the velocity smoother.

---

## Coordinate reference: Gazebo frame vs. map frame

**These are two different coordinate systems — Nav2 goals must always use `map` frame.**

| | Gazebo (world) frame | `map` frame |
|---|---|---|
| Room interior (exact, from `test_room.sdf`) | x: -2.9 to 2.9, y: -2.9 to 2.9 | x: -5.75 to 0.05, y: -3.19 to 2.61 |
| Safe zone (minus 0.7m wall buffer: `robot_radius 0.3` + `inflation_radius 0.4`) | x: -2.2 to 2.2, y: -2.2 to 2.2 | x: -5.05 to -0.65, y: -2.49 to 1.91 |
| Approximate conversion | — | `map_x ≈ gazebo_x − 2.85`, `map_y ≈ gazebo_y − 0.29` |

Rotation between the two frames is negligible (pure position offset, no meaningful yaw difference).

**Caveat:** this offset was derived from a single comparison reading, not independently re-verified. Treat it as "which area to aim for," not a guaranteed formula.

**Ground-truth method (use this whenever precision actually matters):** RViz2 → **Publish Point** tool, click directly on the rendered map, read the real coordinate from `ros2 topic echo /clicked_point` in a terminal. No conversion, no assumptions — reads the actual value straight from the system.

**Before sending any goal, always confirm current position first:**
```bash
ros2 run tf2_ros tf2_echo map base_link
```
If it's within ~0.7m of any wall (in `map` frame, using the table above), drive it to open space manually before sending a goal — a goal sent from too close to a wall will fail regardless of how valid the destination itself is.
