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
            --volume ${HOME}/Workspace/fsm_cartesio:/home/forest_ws/src/fsm_cartesio \
            opensot_fsm
