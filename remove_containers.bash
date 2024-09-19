
if [ -z "$1" ]; then
    export remove_count=1
else 
    export remove_count=$1
fi

export base_name=dev_ws_container

docker ps -a --format "{{.Names}}" | awk '/dev_ws_container6/ {print $1}' | sort -r | head -n $remove_count | xargs docker rm