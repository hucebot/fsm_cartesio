#!/bin/bash
IsRunning=`docker ps -f name=cartesio_tiago_dual | grep -c "cartesio"`;
if [ $IsRunning -eq "0" ]; then
    xhost +local:docker
    docker run --rm \
        -e DISPLAY=$DISPLAY \
        -e XAUTHORITY=$XAUTHORITY \
        -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
        -e 'QT_X11_NO_MITSHM=1' \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /tmp/docker_share:/tmp/docker_share \
        -v `pwd`/..:/home/forest_ws/src/fsm_cartesio \
        -v `pwd`/../../tiago_dual_cartesio_config:/home/forest_ws/src/tiago_dual_cartesio_config \
        -v `pwd`/../../basic_tasks/simple_goto:/home/forest_ws/src/simple_goto \
        --ipc host \
        --device /dev/dri \
        --device /dev/snd \
        --device /dev/input \
        --device /dev/bus/usb \
        --privileged \
        --ulimit rtprio=99 \
        --net host \
        --name cartesio_tiago_dual \
        --entrypoint /bin/bash \
        -ti cartesio:tiago_dual
else
    echo "Docker image is already running. Opening new terminal...";
    docker exec -ti cartesio_tiago_dual /bin/bash
fi
