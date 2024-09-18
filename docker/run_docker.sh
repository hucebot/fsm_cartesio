# For graphics
xhost +

docker run \
            --interactive \
            --tty \
            --rm \
            --env DISPLAY=$DISPLAY \
            --net host \
            --privileged \
            --volume /tmp/.X11-unix:/tmp/.X11-unix \
            --name opensot_fsm \
            opensot_fsm
