
export TurtleId=3

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
<type1> <var1>
<type2> <var2>
<type3> <var3>
string name # Optional.  A unique name will be created and returned if this is empty
---
<type5> <var5>
```


Step X: call the spawn service in YAML syntax
```bash
ros2 service call /spawn <enter_spawn_type_from_step_X> "{<var1>: 2, <var2>: 2, <var3>: 0.2, name: 'turtle$TurtleId'}"
```






ros2 run turtlesim spawn
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle$turtle_id/cmd_vel:=turtle$turtle_id/cmd_vel
```

