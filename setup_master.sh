
source ../../devel/setup.bash

if [[ $# > 0 ]]; then
    export ROS_MASTER_URI=http://localhost:$((11311 + $1))
    export GAZEBO_MASTER_URI=http://localhost:$((11345 + $1))
fi
