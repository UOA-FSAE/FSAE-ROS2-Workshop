export ssh_port=50000

export ros_domain_id=24
export base_name=dev_ws_container

if [ -z "$(docker ps | grep $ssh_port)" ]; then
    docker run --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v $PWD:/devs_ws --name $base_name$ssh_port -d -p $ssh_port:22 devs_ws:host
    echo container at $base_name$ssh_port now starting
    
    # Environment variables set through docker run's -e flag are stored in
    # memory and do not persist. Writing to the ./bashrc allows ssh to
    # access them.
    docker exec $base_name$ssh_port bash -c "echo 'export TURTLE_NAME=turtleHOST' >> ~/.bashrc"
    docker exec $base_name$ssh_port bash -c "echo 'export ROS_DOMAIN_ID=$ros_domain_id' >> ~/.bashrc"
else 
    echo Port $ssh_port occupied - container $base_name$ssh_port could already be running 
fi
