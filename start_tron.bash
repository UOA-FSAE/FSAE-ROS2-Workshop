
ros2 service call /tron_game/set_tron_bike  turtle_tron/srv/SetTronBike "{player: turtleHOST, type: 'chaser'}"

ros2 service call /spawn host "{x: 2, y: 2, theta: 0.2, name: 'turtleHost'}"

ros2 run turtle_tron tron_game