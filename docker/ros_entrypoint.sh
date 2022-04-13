#!/bin/bash
set -e

echo BPO is running now
#fiware setup
sed -e "s/LOCALHOST/$HOST/g" -e "s/FIWAREHOST/$FIWAREHOST/g" -e "s/NETINTERFACE/$NETINTERFACE/g" /root/catkin_ws/src/firos/config/config.json.template > /root/catkin_ws/src/firos/config/config.json


# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/root/catkin_ws/devel/setup.sh"

exec roslaunch opil_bpo bpo_service_v5.launch

exec  "$@"

