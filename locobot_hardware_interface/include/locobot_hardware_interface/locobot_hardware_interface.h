#ifndef __LOCOBOT_HW_INTF_H
#define __LOCOBOT_HW_INTF_H

#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/robot_hw.h"
#include "ros/ros.h"

class LocobotHW: public hardware_interface::RobotHW
{
    public:
        LocobotHW();
        bool init(ros::NodeHandle& /*root_nh*/, ros::NodeHandle& /*robot_hw_nh*/);
        void read(const ros::Time& /*time*/, const ros::Duration& /*period*/);
        void write(const ros::Time& /*time*/, const ros::Duration& /*period*/);

    private:
        static const size_t N_JOINTS = 8;
        static const std::string JNT_NAMES[N_JOINTS];
        static const uint8_t JNT_IDS[N_JOINTS];
        static const uint8_t SHOULDER_SHADOW = 3;
        double jnt_pos[N_JOINTS] = {0.0};
        double jnt_vel[N_JOINTS] = {0.0};
        double jnt_eff[N_JOINTS] = {0.0};
        double jnt_cmd[N_JOINTS] = {0.0};
        hardware_interface::JointStateInterface jnt_state_intf;
        hardware_interface::PositionJointInterface jnt_cmd_intf;
        std::unique_ptr<dynamixel::PortHandler> portHandler;
        std::unique_ptr<dynamixel::PacketHandler> packetHandler;

        static const uint16_t TORQUE_ENABLE_ADDR = 64;
        static const uint16_t READ_POS_ADDR = 132;
        static const uint16_t READ_VEL_ADDR = 128;
        static const uint16_t READ_LOAD_ADDR = 126;
        static const uint16_t WRITE_POS_ADDR = 116;
};

#endif // __LOCOBOT_HW_INTF_H
