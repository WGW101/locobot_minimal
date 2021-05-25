#include "locobot_hardware_interface/locobot_hardware_interface.h"
#include "ros/console.h"
#include <cmath>

const std::string LocobotHW::JNT_NAMES[] = {"waist", "shoulder", "elbow",
                                            "wrist_angle", "wrist_rotate",
                                            "gripper",
                                            "head_pan_joint", "head_tilt_joint"};
const uint8_t LocobotHW::JNT_IDS[] = {1, 2, 4,
                                      5, 6,
                                      7,
                                      8, 9};

LocobotHW::LocobotHW()
{
    for(size_t i = 0; i < N_JOINTS; ++i)
    {
        hardware_interface::JointStateHandle state_handle(JNT_NAMES[i],
                                                          &jnt_pos[i],
                                                          &jnt_vel[i],
                                                          &jnt_eff[i]);
        jnt_state_intf.registerHandle(state_handle);
        hardware_interface::JointHandle handle(jnt_state_intf.getHandle(JNT_NAMES[i]),
                                               &jnt_cmd[i]);
        jnt_cmd_intf.registerHandle(handle);
    }

    registerInterface(&jnt_state_intf);
    registerInterface(&jnt_cmd_intf);
}

bool LocobotHW::init(ros::NodeHandle& /*root_nh*/, ros::NodeHandle& robot_hw_nh)
{
    std::string port;
    robot_hw_nh.param<std::string>("port", port, "/dev/ttyDXL");
    portHandler.reset(dynamixel::PortHandler::getPortHandler(port.c_str()));
    packetHandler.reset(dynamixel::PacketHandler::getPacketHandler(2.0));
    if(!portHandler->openPort())
    {
        ROS_ERROR_STREAM("Failed to open port '" << port << "'");
        return false;
    }

    int baudrate;
    robot_hw_nh.param<int>("baudrate", baudrate, 1000000);
    if(!portHandler->setBaudRate(baudrate)){
        ROS_ERROR_STREAM("Failed to set baudrate '" << baudrate << "'");
        return false;
    }

    int res;
    uint8_t error;
    for(size_t i = 0; i < N_JOINTS; ++i)
    {
        res = packetHandler->write1ByteTxRx(portHandler.get(), JNT_IDS[i],
                                            TORQUE_ENABLE_ADDR, 1, &error);
        if(res != COMM_SUCCESS || error != 0){
            ROS_ERROR_STREAM("Failed to enable torque for joint '" << JNT_NAMES[i] << "'");
            return false;
        }
    }

    ROS_INFO_STREAM("Successfully initialized locobot HW");
    return true;
}

void LocobotHW::read(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
    int res;
    uint8_t error;
    uint32_t pos_val;
    for(size_t i = 0; i < N_JOINTS; ++i)
    {
        res = packetHandler->read4ByteTxRx(portHandler.get(), JNT_IDS[i],
                                           READ_POS_ADDR, &pos_val, &error);
        if(res != COMM_SUCCESS || error != 0)
        {
            jnt_pos[i] = std::nan("");
        }
        else
        {
            jnt_pos[i] = 0.001534355 * pos_val - M_PI;
        }
    }
    int32_t vel_val;
    for(size_t i = 0; i < N_JOINTS; ++i)
    {
        res = packetHandler->read4ByteTxRx(portHandler.get(), JNT_IDS[i],
                                           READ_VEL_ADDR, (uint32_t*)&vel_val, &error);
        if(res != COMM_SUCCESS || error != 0)
        {
            jnt_vel[i] = std::nan("");
        }
        else
        {
            jnt_vel[i] = 0.036446482 * vel_val;
        }
    }
    int16_t load_val;
    for(size_t i = 0; i < N_JOINTS; ++i)
    {
        res = packetHandler->read2ByteTxRx(portHandler.get(), JNT_IDS[i],
                                           READ_LOAD_ADDR, (uint16_t*)&load_val, &error);
        if(res != COMM_SUCCESS || error != 0)
        {
            jnt_eff[i] = std::nan("");
        }
        else
        {
            if(i < 4)
            {
                jnt_eff[i] = 0.004795217 * load_val;
            }
            else
            {
                jnt_eff[i] = 0.0015 * load_val;
            }
        }
    }
    res = packetHandler->read2ByteTxRx(portHandler.get(), SHOULDER_SHADOW,
                                       READ_LOAD_ADDR, (uint16_t*)&load_val, &error);
    if(res != COMM_SUCCESS || error != 0)
    {
        jnt_eff[2] = std::nan("");
    }
    else
    {
        jnt_eff[2] -= 0.004795217 * load_val;
    }
}

void LocobotHW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
    uint8_t error;
    uint32_t pos_val;
    for(size_t i = 0; i < N_JOINTS; ++i)
    {
        pos_val = (uint32_t)(651.739492 * (jnt_cmd[i] + M_PI));
        packetHandler->write4ByteTxRx(portHandler.get(), JNT_IDS[i],
                                      WRITE_POS_ADDR, pos_val, &error);
    }
    pos_val = (uint32_t)(651.739492 * (M_PI - jnt_cmd[2]));
    packetHandler->write4ByteTxRx(portHandler.get(), SHOULDER_SHADOW,
                                  WRITE_POS_ADDR, pos_val, &error);
}
