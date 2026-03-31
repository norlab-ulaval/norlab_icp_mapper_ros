#!/bin/bash

# update this var
ROS_BAG_PATH="/Users/mbo/Documents/norlab/mtt-com/data/COM_Shift_2026_03_27-15_01_34"

docker run --rm -it -v $ROS_BAG_PATH:/rosbag mtt-mapping
