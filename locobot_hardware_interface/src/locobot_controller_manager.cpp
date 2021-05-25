#include "controller_manager/controller_manager.h"
#include "ros/ros.h"

#include "locobot_hardware_interface/locobot_hardware_interface.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "locobot_controller_manager");
    ros::NodeHandle root_nh;
    ros::NodeHandle priv_nh("~");

    LocobotHW locobot;
    if (!locobot.init(root_nh, priv_nh)){ return 1; }

    controller_manager::ControllerManager cm(&locobot, root_nh);

    ros::Rate rate(30);
    ros::Time last_time = ros::Time::now();
    ros::Time cur_time;
    ros::Duration elapsed;

    while(ros::ok())
    {
        cur_time = ros::Time::now();
        elapsed = cur_time - last_time;

        locobot.read(cur_time, elapsed);
        cm.update(cur_time, elapsed);
        locobot.write(cur_time, elapsed);

        last_time = cur_time;
        rate.sleep();
    }
    return 0;
}
