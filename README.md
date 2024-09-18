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

5. Set up the docker environments (on each host machine) by running
   `./start_container.bash <NUMBER_OF_CONTAINERS>`. 
   * Each container has port forwarded to it's ssh service begining from 60000.
   * The ssh password to each container is `WS_PSWD` set previously.
   * The `root` user is fine for attendees to use. This gives them
     sudo-privileges.

6. Distribute the host machine's IP(s) and forwarded ssh ports to each attendee.
   * ssh commands are as followed `ssh root@<host_ip> -p <ssh_port>`
   * distribute the password

7. asd
8. 


when stopped the workspace is **not removed** in case a user wants to resume.