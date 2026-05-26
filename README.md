# behavior_tree_for_damvi

Behavior Tree C++ version for DAMVI.

## Build

After changing BT visualization code:

```bash
cd /home/symoon/Desktop/F1/behavior_tree_for_damvi-behavior_tree_ver2
source /opt/ros/humble/setup.bash
colcon build --packages-select behavior_tree_cpp --symlink-install
source install/setup.bash
```

## Common Source Setup

Use this in each new terminal before testing:

```bash
source /opt/ros/humble/setup.bash
source /home/symoon/Desktop/F1/path_loader-main/install/setup.bash
source /home/symoon/Desktop/F1/local_ws-main/install/setup.bash
source /home/symoon/Desktop/F1/behavior_tree_for_damvi-behavior_tree_ver2/install/setup.bash
source /home/symoon/Desktop/F1/PPcontroller-main/install/setup.bash
```

## BT Mode Test

`obstacle_mode` values:

- `0`: auto
- `1`: no obstacle
- `2`: dynamic only
- `3`: dynamic + static

Overlap handling uses `dynamic_static_overlap_m` (`1.0m` by default):

- mode `0` and `2`: if dynamic/static overlap, dynamic wins and static is suppressed.
- mode `3`: overlap suppression is not applied; dynamic and static flags can both stay active.

Dynamic judgment:

- mode `2` keeps the original dynamic criteria: `/dynamic_obstacle` fresh, in front, and within `thresh_m` (`15.0m` by default).
- mode `3` can add a speed gate to reduce false dynamic judgment on nearly-static objects. It is off by default with `dynamic_min_speed_mps:=0.0`; set it to `0.05` or `0.10` only when detector velocity is reliable.

Run BT:

```bash
ros2 launch behavior_tree_cpp bt_launch.py bypass_critical:=true obstacle_mode:=0
```

Change only the mode number for forced tests:

```bash
ros2 launch behavior_tree_cpp bt_launch.py bypass_critical:=true obstacle_mode:=1
ros2 launch behavior_tree_cpp bt_launch.py bypass_critical:=true obstacle_mode:=2
ros2 launch behavior_tree_cpp bt_launch.py bypass_critical:=true obstacle_mode:=3
ros2 launch behavior_tree_cpp bt_launch.py bypass_critical:=true obstacle_mode:=3 dynamic_min_speed_mps:=0.05
```

Fake-topic tester:

```bash
ros2 run behavior_tree_cpp behavior_tree_tester
```

Tester scenario changes:

```bash
ros2 param set /state_machine_tester scenario_idx 0  # clear
ros2 param set /state_machine_tester scenario_idx 1  # dynamic obstacle ~2m
ros2 param set /state_machine_tester scenario_idx 2  # static obstacle ~2m
ros2 param set /state_machine_tester scenario_idx 3  # dynamic obstacle ~0.9m
ros2 param set /state_machine_tester scenario_idx 4  # static obstacle ~0.9m
```

Useful topic checks:

```bash
ros2 topic echo /obstacle_mode
ros2 topic echo /obj_flag
ros2 topic echo /selected_path --once
ros2 topic echo /bt_decision_marker
```

`/obj_flag.point.x=1` means dynamic obstacle is active. `/obj_flag.point.y=1` means static obstacle is active.

## RF Cartographer Restart

This feature restarts Cartographer from the behavior tree when the mapped RF channel is turned on.

Current RF rule:

- Topic: `/rf`
- Type: `std_msgs/msg/UInt16MultiArray`
- Channel: `data[9]` (`10th` RF value)
- Off: `data[9] <= 1500`
- On / restart trigger: `data[9] > 1500`

The restart target is:

```bash
cd /home/symoon/Desktop/F1/Local_SLAM_Complete/good/SLAM_main-local_loss_wheel
source install/setup.bash
ros2 launch cartographer_ros Damvi_carto_pure_wheel_launch.py
```

When the RF value goes from off to on, BT runs:

```bash
pkill -SIGINT -f 'ros2 launch cartographer_ros Damvi_carto_pure_wheel_launch.py' || true
```

Then it starts Cartographer again in the background. The relaunched Cartographer output goes to:

```bash
/tmp/cartographer_restart.log
```

Run with RF restart enabled:

```bash
ros2 launch behavior_tree_cpp bt_launch.py \
  bypass_critical:=true \
  cartographer_restart_enabled:=true
```

The defaults are already set to channel `9` and threshold `>1500`, so these arguments are optional:

```bash
cartographer_restart_rf_channel:=9
cartographer_restart_rf_min:=1501
cartographer_restart_rf_max:=65535
```

### Test With Bag And Manual RF Trigger

Terminal 1: start the currently used Cartographer.

```bash
cd /home/symoon/Desktop/F1/Local_SLAM_Complete/good/SLAM_main-local_loss_wheel
source install/setup.bash
ros2 launch cartographer_ros Damvi_carto_pure_wheel_launch.py
```

Terminal 2: start BT with restart enabled.

```bash
cd /home/symoon/Desktop/F1/behavior_tree_for_damvi-behavior_tree_ver2
source install/setup.bash

ros2 launch behavior_tree_cpp bt_launch.py \
  bypass_critical:=true \
  cartographer_restart_enabled:=true
```

Terminal 3: play the bag if needed.

```bash
ros2 bag play /home/symoon/Desktop/F1/Local_SLAM_Complete/SLAM_main-local_loss_wheel/0518_2
```

Terminal 4: publish an RF on signal to `/rf.data[9]`.

```bash
cd /home/symoon/Desktop/F1/behavior_tree_for_damvi-behavior_tree_ver2
source install/setup.bash
ros2 run behavior_tree_cpp rf_restart_trigger
```

Expected BT log:

```text
Cartographer restart requested by RF channel 9
Restarting cartographer launch now
cartographer launch restarted
```

Check the restarted Cartographer:

```bash
tail -f /tmp/cartographer_restart.log
pgrep -af "Damvi_carto_pure_wheel_launch.py|cartographer_node"
```

To inspect the live RF value:

```bash
ros2 topic echo /rf --once
```

If the actual controller is mapped correctly, turning the mapped switch on should make `/rf.data[9]` greater than `1500`; turning it off should make it `1500` or lower.

## 0501 Full Pipeline Test

Terminal 1: global path from `/home/symoon/Desktop/F1/path_loader-main/path/0501.csv`.

```bash
ros2 run path_loader path_loader
```

Terminal 2: 0501 static map.

```bash
ros2 run localplanner_cpp static_map_publisher --ros-args \
  -p map_base_dir:=/home/symoon/Desktop/F1/local_ws-main/src/localplanner_cpp/maps \
  -p map_name:=0501.yaml \
  -p topic:=/static_map
```

Terminal 3: local planner.

```bash
ros2 run localplanner_cpp local_planner_node --ros-args \
  --params-file /home/symoon/Desktop/F1/local_ws-main/src/localplanner_cpp/config/params.yaml
```

Terminal 4: behavior tree.

```bash
ros2 launch behavior_tree_cpp bt_launch.py bypass_critical:=true obstacle_mode:=0
```

Terminal 5: pure pursuit using BT-selected path.

```bash
ros2 run pure_pursuit pure_pursuit --ros-args \
  --params-file /home/symoon/Desktop/F1/PPcontroller-main/config/config.yaml \
  -r /Path:=/selected_path
```

Do not run `behavior_tree_tester` during this full pipeline test because it publishes fake `/odom`, `/Path`, `/global_path`, and obstacle topics.

## RViz Visualization

Set RViz `Fixed Frame` to `map`.

Add these displays:

- `Map`: `/static_map`
- `Path`: `/global_path`
- `Path`: `/Path`
- `Path`: `/selected_path`
- `Marker`: `/local_path`
- `Marker`: `/bt_decision_marker`

`/bt_decision_marker` is published by `bt_main`. It shows:

- `NO OBSTACLE`
- `DYNAMIC`
- `STATIC`
- `DYNAMIC + STATIC`
