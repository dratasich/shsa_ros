# ROS Wrapper for SHSA
Self-Healing by Structural Adaptation (SHSA) for the Robot Operating System
(ROS).

This repo implements the ROS interface for
the [shsa library] and [shsa-problog library],
to demonstrate SHSA on a mobile robot.


## Dependencies

The ROS nodes `shsa_node.py`, `transfer_function_node.py`, `watchdog_node.py`
depend on the [shsa library].

The ROS node `monitor_node.py`
depends on the [shsa-problog library].

Note that the libraries cannot be both in `PYTHONPATH` (name clash).
We used [docker] containers for the ROS nodes anyway.


## Run

Execute SHSA node:
```bash
rosrun shsa_ros shsa_node.py _model:=/path/to/shsa-model.yaml &
```

Substitute a topic (send a goal, i.e., trigger an action of the SHSA node):
```bash
rostopic pub -1 /substitute/goal shsa_ros/SubstituteActionGoal
```

Trigger auto-completion by pressing `tab` two times, the content of the goal
message can be filled (`goal.topic`).


## Demo

This package also includes application launch files,
e.g., to demonstrate SHSA on the mobile robot [Daisy]().

To run the demo you will need additional ROS and python packages.

### Collision Avoidance with Daisy

Get the [shsa library] and install the python packages needed
(see the `requirements.txt` file).

Add the library to `PYTHONPATH`.

Source the catkin workspace containing
[general-ros-modules](https://github.com/tuw-cpsg/general-ros-modules) and
[shsa_ros](https://github.com/dratasich/shsa_ros).

Run the application:
```bash
roslaunch shsa_ros rover.launch
```

In case the motors haven't been enabled
(a start after p2os_driver is necessary),
launch `enablemotors.launch`.

Then start the watchdog and SHSA.
```bash
roslaunch shsa_ros shsa.launch
```

When the laser breaks (e.g., switch off the power supply),
the `watchdog_node` triggers the `shsa_node`
to substitute the minimum-distance-to-an-obstacle (`dmin`) calculation.


[shsa library]: https://github.com/dratasich/shsa
[shsa-problog library]: https://github.com/dratasich/shsa-problog
[docker]: https://github.com/dratasich/docker
