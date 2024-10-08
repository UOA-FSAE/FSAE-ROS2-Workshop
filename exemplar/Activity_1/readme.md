# Activity 1 - interacting with the Turtlesim
Learning to use the ROS 2 CLI 

## Instructions
to set your own TURTLE_NAME use the following command. Otherwise you have been
given a unique TURTLE_NAME co-responding to your container
```bash
export TURTLE_NAME=3
```

For the purpose of this  

Show all services
```bash
ros2 service list
```

Step X: get the type of message used by the spawn service
```bash
ros2 service type /spawn
```



additionally you could list all types
```bash
ros2 service list -t
```

Step X: get the interface of /spawn
```bash
ros2 interface show <enter_spawn_type_from_step_X>
```

You should see something of the following form:
```bash
float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name
```


Step X: call the spawn service in YAML syntax
```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: '$TURTLE_NAME'}"
```


Step X: start a teleop process
```bash
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=$TURTLE_NAME/cmd_vel
```

Follow the prompts in your terminal to control your turtle up on the screen.

### Additional steps

Step X: teleport your turtle
```bash

```



