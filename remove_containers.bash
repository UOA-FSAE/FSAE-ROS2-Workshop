
if [ -z "$1" ]; then
    export remove_count=1
else 
    export remove_count=$1
fi

docker ps -a --format "{{.Names}}" | awk '/ssh-test6/ {print $1}' | sort -r | head -n $remove_count | xargs docker rm