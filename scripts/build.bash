#!/bin/bash

cd ../..
colcon build --packages-select my_web_package 
source ./install/local_setup.bash
ros2 launch my_web_package wgg.launch.py