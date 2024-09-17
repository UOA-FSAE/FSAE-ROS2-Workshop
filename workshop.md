# ROS 2 workshop reference
This is a quick reference guide for use in running the workshop activities. 



## Activity 1

start host container

```bash
./start_host_container.bash
```

start attendees containers
```bash
./start_container.bash <number_of_required_containers> [<starting_port>]
```

launch the turtlesim activity
```bash
ros2 run turtlesim turtlesim_node 
```