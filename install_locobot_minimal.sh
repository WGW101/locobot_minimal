#!/bin/bash


source /opt/ros/noetic/setup.bash

wstool merge https://raw.githubusercontent.com/wgw101/locobot_minimal/master/locobot_minimal.rosinstall
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

rm -r kobuki/kobuki \
      kobuki/kobuki_bumper2pc \
      kobuki/kobuki_controller_tutorial \
      kobuki/kobuki_testsuite
rm -r yujin_ocs/yocs_ar_marker_tracking \
      yujin_ocs/yocs_ar_pair_approach \
      yujin_ocs/yocs_ar_pair_tracking \
      yujin_ocs/yocs_diff_drive_pose_controller \
      yujin_ocs/yocs_joyop \
      yujin_ocs/yocs_keyop \
      yujin_ocs/yocs_localization_manager \
      yujin_ocs/yocs_math_toolkit \
      yujin_ocs/yocs_navigator \
      yujin_ocs/yocs_navi_toolkit \
      yujin_ocs/yocs_rapps \
      yujin_ocs/yocs_safety_controller \
      yujin_ocs/yocs_virtual_sensor \
      yujin_ocs/yocs_waypoint_provider \
      yujin_ocs/yocs_waypoints_navi \
      yujin_ocs/yujin_ocs
rm -r interbotix_ros_core/interbotix_ros_uxarms/
rm interbotix_ros_core/interbotix_ros_xseries/CATKIN_IGNORE
rm -r interbotix_ros_toolboxes/interbotix_ux_toolbox/
rm interbotix_ros_toolboxes/interbotix_xs_toolbox/CATKIN_IGNORE
rm interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/CATKIN_IGNORE
rm -r interbotix_ros_core/interbotix_ros_xseries/dynamixel_workbench_toolbox/
rm -r interbotix_ros_manipulators/interbotix_ros_uxarms/
rm interbotix_ros_manipulators/interbotix_ros_xsarms/CATKIN_IGNORE

rosdep update
rosdep install -i -r -y --from-paths .
catkin_make -C .. -j 4
source ../devel/setup.bash

curl 'https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules' \
	> /etc/udev/rules.d/99-realsense-libusb.rules
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{Product}=="CP210* USB to UART*", MODE:="0666", SYMLINK+="ttyLDS01"' \
	> /etc/udev/rules.d/99-hls-lfcd-lds.rules
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{Product}=="USB <-> Serial*", ATTR{device/latency_timer}="1", MODE:="0666", GROUP:="dialout", SYMLINK+="ttyDXL"' \
	> /etc/udev/rules.d/99-dynamixel-u2d2.rules
