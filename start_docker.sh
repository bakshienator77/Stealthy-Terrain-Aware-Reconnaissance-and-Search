docker pull osrf/ros:melodic-desktop-full
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
docker run -it --gpus=all --privileged --log-driver=syslog --net=host `echo -v /tmp:/tmp:rw -v $SCRIPT_DIR/src:/home/user/src -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /tmp/.docker.xauth:/tmp/.docker.xauth -e DISPLAY=$DISPLAY -e XAUTHORITY=/tmp/.docker.xauth` --rm --name ros-docker-py2.7 osrf/ros:melodic-desktop-full  bash
