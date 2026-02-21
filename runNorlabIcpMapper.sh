#!/bin/bash

docker run -it \
  --name slam \
  -it \
  --rm \
  --privileged \
  --network host \
  -e SSH_AUTH_SOCK=${SSH_AUTH_SOCK} \
  -v /home/mabox/data/2024-11-21/red_2024-11-21-10-34:/data \
  -v /home/mabox/data/2024-11-21/red_2024-11-21-10-34/calib/:/calib \
  -e NAMESPACE="" \
  -e IS_MAPPING=1 \
  -e STORAGE_PATH=/ros2_ws/src/norlab_icp_mapper_ros/output \
  norlab/norlabicpmapper_offline /bin/bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch norlab_icp_mapper_ros offline_mapper.launch.py bag_path:=/data"

  