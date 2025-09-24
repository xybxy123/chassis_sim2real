#include "chassis_hw/chassis_hw.h"
#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <cmath>
#include <algorithm>  

namespace chassis_hw
{

ChassisHW::~ChassisHW()
{
  // 析构函数中关闭CAN设备
  if (usb_can_)
  {
    usb_can_->close();
  }
}

bool ChassisHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
  // 读取参数，带默认值
  robot_hw_nh.param<std::string>("can_interface", can_interface_, "can0");
  robot_hw_nh.param<int>("can_thread_priority", can_thread_priority_, 50);
  
  // 声明 XmlRpcValue 变量
  XmlRpc::XmlRpcValue motor_ids_xml;
  
  // 获取电机ID参数
  if (!root_nh.getParam("/chassis/motor_ids", motor_ids_xml) || 
      motor_ids_xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Failed to get parameter 'motor_ids' or it's not an array");
    return false;
  }
  
  // 解析电机ID
  for (int i = 0; i < motor_ids_xml.size(); ++i)
  {
    motor_ids_.push_back(static_cast<int>(motor_ids_xml[i]));
    joint_names_[motor_ids_.back()] = "motor_" + std::to_string(motor_ids_.back()) + "_joint";
  }
  
  if (motor_ids_.empty())
  {
    ROS_ERROR("No motor IDs specified");
    return false;
  }
  
  // 初始化USB-CAN设备
  usb_can_ = std::make_unique<UsbCan>();
  if (!usb_can_->init(can_interface_, motor_ids_, can_thread_priority_))
  {
    ROS_ERROR("Failed to initialize USB-CAN device");
    return false;
  }
  
  // 为每个电机创建接口
  for (int id : motor_ids_)
  {
    const std::string& joint_name = joint_names_[id];
    
    // 初始化电机数据
    motor_data_[id] = {0.0, 0.0, 0.0, 0.0, 0.0};
    motor_commands_[id] = {0.0};
    
    // 注册关节状态接口
    hardware_interface::JointStateHandle state_handle(
      joint_name, 
      &motor_data_[id].angle, 
      &motor_data_[id].velocity, 
      &motor_data_[id].effort
    );
    joint_state_interface_.registerHandle(state_handle);
    
    // 注册关节命令接口
    hardware_interface::JointHandle effort_handle(
      joint_state_interface_.getHandle(joint_name),
      &motor_commands_[id].effort
    );
    effort_joint_interface_.registerHandle(effort_handle);
  }
  
  // 注册接口
  registerInterface(&joint_state_interface_);
  registerInterface(&effort_joint_interface_);
  
  ROS_INFO("ChassisHW initialized with %zu motors on CAN interface %s", 
           motor_ids_.size(), can_interface_.c_str());
  return true;
}

void ChassisHW::read(const ros::Time& time, const ros::Duration& period)
{
  std::lock_guard<std::mutex> lock(can_mutex_);
  
  // 读取每个电机的数据
  for (int id : motor_ids_)
  {
    MotorData data = usb_can_->getMotorData(id);
    
    motor_data_[id].angle = data.angle;
    motor_data_[id].velocity = data.velocity;
    motor_data_[id].effort = data.effort;
    motor_data_[id].temp = data.temp;

    
  }
}

void ChassisHW::write(const ros::Time& time, const ros::Duration& period)
{
  std::lock_guard<std::mutex> lock(can_mutex_);
  
  // 发送每个电机的命令
  for (int id : motor_ids_)
  {
    MotorCommand cmd = motor_commands_[id];
    
    const double MAX_EFFORT = 30.0; 
    cmd.effort = std::clamp(cmd.effort, -MAX_EFFORT, MAX_EFFORT);
    
    usb_can_->sendMotorCommand(id, cmd);
  }
}

} // namespace chassis_hw