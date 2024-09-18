

if [ -z "$1" ]; then
    export user_count=0
else 
    export user_count=$(expr $1 - 1)
fi

if [ -z "$2" ]; then
    export ssh_first_port=60000
else 
    export ssh_first_port=$2
fi

$ros_domain_id = 24


for i in $( seq 0 $user_count ); do
    export ssh_port=$(expr $ssh_first_port + $i)
    if [ -z "$(docker ps | grep $ssh_port)" ]; then
        docker run -v $PWD/install:/devs_ws/install -e TurtleName=turtle$i -e ROS_DOMAIN_ID=$ros_domain_id --name ssh-test$ssh_port -d -p $ssh_port:22 devs_ws:copy_fs > /dev/null 2>&1
        echo container at $ssh_port now starting
    else 
        echo container at $ssh_port already running
    fi
done
