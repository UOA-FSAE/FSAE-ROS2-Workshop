export ssh_port=50000

if [ -z "$(docker ps | grep $ssh_port)" ]; then
    docker run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v $PWD:/devs_ws --name ssh-test$ssh_port -d -p $ssh_port:22 devs_ws:host
    echo container at $ssh_port now starting
else 
    echo container at $ssh_port already running
fi
