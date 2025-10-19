#!/bin/bash

colcon build 

source install/setup.bash

ros2 run ts_topic ts_topic_node