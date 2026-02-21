#!/bin/bash

docker run -it \
  --name slam \
  -it \
  --rm \
  --privileged \
  --network host \
  -e SSH_AUTH_SOCK=${SSH_AUTH_SOCK} \
  -v ../../ros_launchers:/ros2_ws/src/ros_launchers \
  -v /home/mabox/data:/data \
  -v /home/mabox/norlab_icp_mapper_ros:/ros2_ws/src/norlab_icp_mapper_ros \
  -e NAMESPACE="" \
  -e IS_MAPPING=1 \
  -e STORAGE_PATH=/ros2_ws/src/2fast2lamaa/output \
  norlab/norlabicpmapper /bin/bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && cd /ros2_ws/src/norlab_icp_mapper_ros && colcon build --symlink-install --packages-select norlab_icp_mapper_ros"


  # -v ../../ros_launchers:/root/ASRL/vtr3/src/main/src/ros_launchers \
  # -v ../../ros_launchers:/ros2_ws/src/ros_launchers \
  # -v /home/mbo/bigfoot-FoMo/mcap:/rosbags \