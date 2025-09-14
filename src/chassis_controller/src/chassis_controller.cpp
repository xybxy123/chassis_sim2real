#include "chassis_controller/chassis_controller.h"

namespace chassis_controller{

bool ChassisController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
{
  // 从参数服务器获取关节名称
  if (!n.getParam("left_front_wheel_joint", front_left_joint_name_)) {
    ROS_ERROR("Could not find parameter: front_left_joint");
    return false;
  }
  if (!n.getParam("right_front_wheel_joint", front_right_joint_name_)) {
    ROS_ERROR("Could not find parameter: front_right_joint");
    return false;
  }
  if (!n.getParam("left_back_wheel_joint", back_left_joint_name_)) {
    ROS_ERROR("Could not find parameter: back_left_joint");
    return false;
  }
  if (!n.getParam("right_back_wheel_joint", back_right_joint_name_)) {
    ROS_ERROR("Could not find parameter: back_right_joint");
    return false;
  }

  // 获取关节控制句柄（使用名称变量）
  try {
    front_left_joint_ = hw->getHandle(front_left_joint_name_);
    front_right_joint_ = hw->getHandle(front_right_joint_name_);
    back_left_joint_ = hw->getHandle(back_left_joint_name_);
    back_right_joint_ = hw->getHandle(back_right_joint_name_);
    ROS_INFO("Successfully got all joint handles");
  } catch (const std::exception& e) {
    ROS_ERROR("Failed to get joint handle: %s", e.what());
    return false;
  }

  return true;
}

void ChassisController::starting(const ros::Time& time) {
  // 初始化速度订阅
  vel_sub_ = ros::NodeHandle().subscribe("/cmd_vel", 10, &ChassisController::velCallback, this);
}

void ChassisController::velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  vel_msg_ = *msg;  // 保存速度指令
}

void ChassisController::update(const ros::Time& time, const ros::Duration& period)
{
  // 计算四个轮子的目标速度 (rad/s)
  double linear_x_ = 0.6*vel_msg_.linear.x;    // 线性x方向速度
  double linear_y_ = 0.6*vel_msg_.linear.y;    // 线性y方向速度
  double angular_z_ = 0.2*vel_msg_.angular.z;  // 角速度
  double L = wheel_base_;                  // 轴距（原代码中wheel_base_）
  double W = wheel_track_;                 // 轮距（原代码中wheel_track_）
  double R = wheel_radius_;                // 轮半径（原代码中wheel_radius_）
  
  double front_left_target = (linear_x_ - linear_y_ - angular_z_ * (L/2 + W/2)) / R;
  double front_right_target = (linear_x_ + linear_y_ + angular_z_ * (L/2 + W/2)) / R;
  double back_left_target = (linear_x_ + linear_y_ - angular_z_ * (L/2 + W/2)) / R;
  double back_right_target = (linear_x_ - linear_y_ + angular_z_ * (L/2 + W/2)) / R;

  // 设置关节力指令
  front_left_joint_.setCommand(front_left_target);
  front_right_joint_.setCommand(front_right_target);
  back_left_joint_.setCommand(back_left_target);
  back_right_joint_.setCommand(back_right_target);
}

void ChassisController::stopping(const ros::Time& time) {
  // 停止时归零指令
  front_left_joint_.setCommand(0.0);
  front_right_joint_.setCommand(0.0);
  back_left_joint_.setCommand(0.0);
  back_right_joint_.setCommand(0.0);
}

}//namespace

// 注册控制器插件
PLUGINLIB_EXPORT_CLASS(chassis_controller::ChassisController, controller_interface::ControllerBase)
