#!/bin/bash
rosbag record --lz4 -b 2048 /oak_ros/oak1/imu /oak_ros/oak1/cama/image_rect_raw /oak_ros/oak1/camb/image_rect_raw /oak_ros/oak1/cama/camera_info /oak_ros/oak1/camb/camera_info