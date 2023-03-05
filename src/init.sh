bash /ros_entrypoint.sh
apt update
apt install python-pip -y
cd zone_recon
apt-get update
apt-get install ros-melodic-catkin
apt-get install python-tk -y
pip install catkin-tools
cat requirements.txt | xargs -n 1 pip install  # ignore any failures
catkin build
source devel/setup.bash