# FSAE ROS2 Workshop

This repo is a fun and interactive learning resource made to teach others about
ROS2. This is not a fully comprehensive guide, and it is recommended the workshop
host already knows enough ROS2 in order to run it. Included is a set
of activities that a host can run with a group of people with readmes and
templated source code.

The accompanying slides can be found on our sharepoint drive [here](https://uoafsae.sharepoint.com/:p:/s/F1-Tenth/ESYzbT8jKupJvBHaOtrGQIoBbPC0pqK-0AZhTpZAHME3hw?e=LqYjkY).

This repo is further comprised of the
[/UOA-FSAE/FSAE-ROS2-Workshop-Workspace](https://github.com/UOA-FSAE/FSAE-ROS2-Workshop-Workspace)
repo, within which the activity instructions and templates are found.

Included are utility scripts and dockerfiles for setting up docker containers
for attendees to access through ssh. 



# Workshop Instructions

## Setup (before workshop)
1. Sort out a networking solution
   * If multiple machines are being used as host ensure they are on the same
     local network. 
   * All Attendees should connect to the same network. 
   *  In particular ensure they are on the **same VLAN** (networks with the same
   SSID) and **same subnet**. Ie the network name (UoA-WiFi) and host address
   (the part masked by the Subnet Mask) are the same. This can be checked using
   `ipconfig` on windows or `ifconfig` on linux. Eg. with a Subnet Mask of
   `255.255.254.0` the address `172.23.40.189` is on the `172.23.40.0` subnet.
   * It is highly unlikely everyone will be allocated to the same subnet via a
     university router.
   * For more control, **set up a local network using a FSAE router** to have
     everyone (including the host devices) communicate over. 

2. This repo should be cloned onto the device(s) which will host the docker
   workspaces. 
   * `mkdir workshop`
   * `cd workshop`
   * `git clone https://github.com/UOA-FSAE/FSAE-ROS2-Workshop`

4. Set container ssh password environment variables
   * Define the password used to access the containers over ssh using `export WS_PSWD=<write_password>` in Linux or `set WS_PSWD=<write_password>` 
   * To view this password against use `echo $WS_PSWD` in linux or `echo %WS_PSWD%` in windows.
   * (Optional) Define the password used to access the host container(s) over ssh using `export HOST_PSWD=<write_password>` in Linux or `set HOST_PSWD=<write_password>` 
   * (Optional) To view this password against use `echo $WS_PSWD` in linux or `echo %WS_PSWD%` in windows.

4. Build the docker images
   * Run `docker build --build-arg WS_PSWD=fsaedevs --build-arg ROS_DOMAIN_ID=24 -t devs_ws:copy_fs -f docker/ws.DockerFile .`
   * Run `docker build --build-arg WS_PSWD=fsaehost47 --build-arg ROS_DOMAIN_ID=24 -t devs_ws:host -f docker/ws_host.DockerFile .`

5. Build packages
   * `colcon build --symlink-install --packages-select turtle_tron turtle_tron_interfaces --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3.8`

6. Set up the docker environments (on each host machine) by running
   `./start_container.bash <NUMBER_OF_CONTAINERS>`. 
   * Each container has port forwarded to it's ssh service begining from 60000.
   * The ssh password to each container is `WS_PSWD` set previously.
   * The `root` user is fine for attendees to use. This gives them
     sudo-privileges.

7. Distribute the host machine's IP(s) and forwarded ssh ports to each attendee.
   * ssh commands are as followed `ssh root@<host_ip> -p <ssh_port>`
   * distribute the password

8. open 2 docker exec instances

9. launch the turtlesim node
   * run `ros2 run turtlesim turtlesim_node --ros-args -p background_r:=255 -p background_g:=255 -p background_b:=255`
   <!-- * to launch a second terminal use the ros-args flag `-r __node:=turtlesim2` -->
10. launch rqt introspection plugin 


## Demonstration (during workshop)

Spawn a turtle from off-screan
`ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: '$TURTLE_NAME'}"`

1. start Tron
2. start a new sim
`ros2 run turtlesim turtlesim_node --ros-args -r __node:=turtlesim_tron -pbackground_r:=255 -p background_g:=255 -p background_b:=255`
run tron node
`ros2 run turtle_tron tron_game`
set team colors
`ros2 service call /tron_game/set_team_color turtle_tron_interfaces/srv/SetTeamColor "{team: 0, r: 255, g: 0, b: 0, width: 10}"`
`ros2 service call /tron_game/set_team_color turtle_tron_interfaces/srv/SetTeamColor "{team: 1, r: 0, g: 255, b: 0, width: 10}"`

assign teams
`ros2 service call /tron_game/assign_teams std_srvs/srv/Empty`


start game
`ros2 service call /tron_game/start std_srvs/srv/Empty`
`ros2 service call /tron_game/start std_srvs/srv/Empty`

respawn players
`ros2 service call /tron_game/respawn_all std_srvs/srv/Empty`







when ws containers are stopped the container is **not removed** in case a user
wants to reconnect and resume.

**The host container is automatically removed when stopped.**



If the readmes are updated, they can be re-exported using the VS Code [Markdown
PDF](https://marketplace.visualstudio.com/items?itemName=yzane.markdown-pdf) extension


If you get the following error you can resolve it by running `ssh-keygen -R
<HOST-IP>:<PORT>`
```
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@    WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED!     @
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
IT IS POSSIBLE THAT SOMEONE IS DOING SOMETHING NASTY!
Someone could be eavesdropping on you right now (man-in-the-middle attack)!
It is also possible that a host key has just been changed.
The fingerprint for the ED25519 key sent by the remote host is
SHA256:lf5fZ9te8RkNRDkVZJMN0ubXc3+CxitpHzI3bdwHWlM.
Please contact your system administrator.
```