# ROS Wrapper for SHSA
Self-Healing by Structural Adaptation (SHSA) for the Robot Operating System
(ROS).


## Dependencies

* [shsa library]

The launch files require additional ROS and python packages (see demos below).

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

### Collision Avoidance with Daisy

Install: tf,
[shsa library] and its dependencies that are the python packages
networkx (>=2.0),
enum34,
future,
pyyaml.

Add the [shsa library] path to `PYTHONPATH`.

Source workspace containing:
[general-ros-modules](https://github.com/tuw-cpsg/general-ros-modules),
[shsa_ros](https://github.com/dratasich/shsa_ros).

```bash
roslaunch shsa_ros rover.launch
```

In case the motors haven't be enabled (a start after p2os_driver is necessary),
launch `enablemotors.launch`.


[shsa library]: https://github.com/dratasich/shsa
