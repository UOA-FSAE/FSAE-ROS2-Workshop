export ssh_port=50000

export base_name=dev_ws_container

if [ -z "$(docker ps | grep $ssh_port)" ]; then
    docker run --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v $PWD:/devs_ws --name $base_name$ssh_port -d -p $ssh_port:22 devs_ws:host
    echo container at $base_name$ssh_port now starting
else 
    echo Port $ssh_port occupied - container $base_name$ssh_port could already be running 
fi
