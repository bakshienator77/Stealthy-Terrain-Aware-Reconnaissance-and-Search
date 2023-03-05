ARG FROM_TAG="dev"
ARG FROM_IMAGE="aidtr-externals"

FROM ${FROM_IMAGE}:${FROM_TAG} as BASE

FROM BASE as aidtr-msgs
ADD . /home/unicorn/src/aidtr-msgs

RUN mkdir -p /tmp/build && sudo mkdir -p /opt/unicorn-deploy && sudo chown unicorn /opt/unicorn-deploy && sudo ldconfig \
&& export PYTHONPATH=/opt/ros/melodic/lib/python2.7/dist-packages/ \
&&  bash -c "cd /tmp/build && source /ros_entrypoint.sh; cmake -G Ninja -DCMAKE_INSTALL_PREFIX=/opt/unicorn-deploy/aidtr-msgs /home/unicorn/src/aidtr-msgs" \
&& cd /tmp/build && ninja install && rm -rf /tmp/build && rm -rf /home/unicorn/.cmake && rm -rf /home/unicorn/.ccache && sudo rm -rf /home/unicorn/src/aidtr-msgs

