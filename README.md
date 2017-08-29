# ROS Wrapper for SHSA
Self-Healing by Structural Adaptation (SHSA) for the Robot Operating System
(ROS).


## Dependencies

* change path to [shsa library](https://github.com/dratasich/shsa) in
  `src/shsa_node.py`
* install `rospkg` for python3
    ```bash
    pip3 install --user rospkg
    ```

## Run

Execute SHSA node:
```bash
rosrun shsa_ros shsa_node.py _model:=/path/to/shsa-model.yaml _map:=/path/to/variable-topic-mapping.yaml &
```

Substitute a topic (send a goal, i.e., trigger an action of the SHSA node):
```bash
rostopic pub -1 /substitute/goal shsa_ros/SubstituteActionGoal
```

Trigger auto-completion by pressing `tab` two times, the content of the goal
message can be filled (`goal.topic`).
