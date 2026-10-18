# Project environment: load it with "source <workspace>/src/dds_leakage/env.sh"
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30          # isolates it from other projects (which use domain 0 by default)
export ROS_LOCALHOST_ONLY=1      # DDS traffic does not leave your machine
# The workspace is two levels above this package (<workspace>/src/dds_leakage)
_DDS_LEAKAGE_WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
[ -f "$_DDS_LEAKAGE_WS/install/setup.bash" ] && source "$_DDS_LEAKAGE_WS/install/setup.bash"
unset _DDS_LEAKAGE_WS
