// chassis_hw.h
#ifndef CHASSIS_HW_H
#define CHASSIS_HW_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "chassis_hw/motor_types.h"
#include <hardware_interface/imu_sensor_interface.h>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <memory>
#include "chassis_hw/usb_can.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace chassis_hw
{

class ChassisHW : public hardware_interface::RobotHW
{
public:
  ChassisHW() = default;
  ~ChassisHW() override;
  
  /**
   * @brief 初始化硬件接口
   * @param root_nh 根节点句柄
   * @param robot_hw_nh 硬件接口节点句柄
   * @return 初始化成功返回true
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh) override;
  
  /**
   * @brief 从硬件读取数据
   * @param time 当前时间
   * @param period 时间间隔
   */
  void read(const ros::Time& time, const ros::Duration& period) override;
  
  /**
   * @brief 向硬件写入命令
   * @param time 当前时间
   * @param period 时间间隔
   */
  void write(const ros::Time& time, const ros::Duration& period) override;

private:
  // 硬件接口
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::EffortJointInterface effort_joint_interface_;
  
  // 电机ID列表
  std::vector<int> motor_ids_;
  
  // 电机数据存储
  std::map<int, MotorData> motor_data_;
  std::map<int, MotorCommand> motor_commands_;
  
  // 关节名称映射
  std::map<int, std::string> joint_names_;
  
  // USB-CAN设备
  std::unique_ptr<chassis_hw::UsbCan> usb_can_;
  
  // 线程安全互斥锁
  std::mutex can_mutex_;
  
  // 参数
  std::string can_interface_;
  int can_thread_priority_;
};
} // namespace chassis_hw

// 注册插件
PLUGINLIB_EXPORT_CLASS(chassis_hw::ChassisHW, hardware_interface::RobotHW)

#endif // CHASSIS_HW_H
