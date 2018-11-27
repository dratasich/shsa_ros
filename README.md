# ROS Wrapper for SHSA
Self-Healing by Structural Adaptation (SHSA) for the Robot Operating System
(ROS).

This repo implements the ROS interface for
the [shsa library] and [shsa-problog library],
to demonstrate SHSA on the mobile robot [Daisy].


## Dependencies

* The ROS nodes `shsa_node.py`, `transfer_function_node.py`, `watchdog_node.py`
  depend on the [shsa library].

* The ROS node `monitor_node.py`
  depends on the [shsa-problog library].

Note that the libraries cannot be both in `PYTHONPATH` (name clash).
We used custom [docker images] to run the ROS nodes.


## Run

Execute SHSA node:
```bash
$ rosrun shsa_ros shsa_node.py _model:=/path/to/shsa-model.yaml &
```

Substitute a topic (send a goal, i.e., trigger an action of the SHSA node):
```bash
$ rostopic pub -1 /substitute/goal shsa_ros/SubstituteActionGoal
```

Trigger auto-completion by pressing `tab` two times, the content of the goal
message can be filled (`goal.topic`).


## Demo

This package also includes application launch files,
e.g., to demonstrate SHSA on the mobile robot [Daisy].

To run the demo you will need additional ROS and python packages.

### Collision Avoidance with Daisy - Breakdown of the Laser

Get the [shsa library] and install the python packages needed
(see the `requirements.txt` file).
Add the library to `PYTHONPATH`.
Source the catkin workspace containing
[general-ros-modules](https://github.com/tuw-cpsg/general-ros-modules) and
[shsa_ros](https://github.com/dratasich/shsa_ros).

Run the application:
```bash
$ roslaunch shsa_ros demo_daisy.launch
```
which starts the drivers and the [teleoperation node](https://github.com/tuw-cpsg/general-ros-modules/tree/master/pioneer_teleop) of Daisy.
Use the keys `w|a|s|d` and `space` to move and stop the robot.
In case the motors haven't been enabled
(a start after `p2os_driver` is necessary),
launch `enablemotors.launch`.

Start the watchdog and self-healing engine with:
```bash
$ roslaunch shsa_ros demo_breakdown.launch
```

When the laser breaks (e.g., switch off the power supply),
the `watchdog_node` triggers the `shsa_node`
to substitute the minimum-distance-to-an-obstacle (`dmin`) calculation.

### Collision Avoidance with Daisy - Spoofing Attack

Get the [shsa library] and [shsa-problog library] and install the python packages needed
(see the `requirements.txt` file).
Add the proper library to `PYTHONPATH` (depends on the nodes to start).
Source the catkin workspace containing
[general-ros-modules](https://github.com/tuw-cpsg/general-ros-modules) and
[shsa_ros](https://github.com/dratasich/shsa_ros).

Run the application:
```bash
$ roslaunch shsa_ros demo_daisy.launch
```

Start the self-healing engine with:
```bash
$ roslaunch shsa_ros shsa.launch
```

Start the monitoring exploiting redundancy:
```bash
$ roslaunch shsa_ros monitor.launch
```

Plot minimum-distance-to-an-obstacle `dmin`:
```bash
$ rqt_plot /emergency_stop/dmin/data /dmin_monitor/value_0/data /dmin_monitor/value_1/data
```

When an attacker spoofs wrong laser data to the ROS network, e.g., by:
```bash
$ rostopic pub /hokuyo/scan sensor_msgs/LaserScan "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
angle_min: 0.0
angle_max: 0.0
angle_increment: 0.0
time_increment: 0.0
scan_time: 0.0
range_min: 0.0
range_max: 0.0
ranges: [3.0]
intensities: [0]" -r 100
```
the `monitor_node` triggers the `shsa_node`
to substitute the minimum-distance-to-an-obstacle (`dmin`) calculation.



[shsa library]: https://github.com/dratasich/shsa
[shsa-problog library]: https://github.com/dratasich/shsa-problog
[docker images]: https://github.com/dratasich/docker
[Daisy]: https://tuw-cpsg.github.io/tutorials/daisy/
