#!/bin/bash

source /opt/ros/noetic/setup.bash

wstool merge https://raw.githubusercontent.com/wgw101/locobot_minimal/locobot_minimal.rosinstall
wstool update

# locobot
#     locobot_bringup
#         kobuki_node
#             kobuki_keyop
#                 yocs_cmd_vel_mux
#                 yocs_velocity_smoother
#             kobuki_safety_controller
#                 yocs_controllers
#             kobuki_rapps
#                 kobuki_auto_docking
#                 kobuki_random_walker
#         dynamixel_workbench
#             dynamixel_workbench_msgs
#     locobot_description
#         kobuki_description

rosdep update
rosdep install -i -r -y --from-paths .
catkin_make -C .. -j 4
source ../devel/setup.bash

curl 'https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules' \
	> /etc/udev/rules.d/99-realsense-libusb.rules
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{Product}=="CP210* USB to UART*", MODE:="0666", SYMLINK+="lds01"' \
	> /etc/udev/rules.d/99-hls-lfcd-lds.rules
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{Product}=="USB <-> Serial*", ATTR{device/latency_timer}="1", MODE:="0666", GROUP:="dialout", SYMLINK+="dynamixel"' \
	> /etc/udev/rules.d/99-dynamixel-u2d2.rules
