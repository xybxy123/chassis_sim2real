#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace chassis_controller{  // 统一命名空间

class ChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;
  

private:
  // 关节句柄（控制用）
  hardware_interface::JointHandle front_left_joint_, front_right_joint_, 
                                 back_left_joint_, back_right_joint_; 
  // 关节名称（配置用，与句柄区分开）
  std::string front_left_joint_name_, front_right_joint_name_, 
              back_left_joint_name_, back_right_joint_name_;
  
  // 物理参数
  double wheel_radius_ = 0.07625;  // 轮子半径
  double wheel_base_ = 0.4;     // 轮距（示例值）
  double wheel_track_ = 0.4;    // 轴距（示例值）
  
  // 速度指令
  ros::Subscriber vel_sub_;
  geometry_msgs::Twist vel_msg_;
  void velCallback(const geometry_msgs::Twist::ConstPtr& msg);  // 回调声明
};

}//namespace

//-- 订阅话题速度 -- 解算成轮子速度 -- 发出轮子速度：串口/话题
// --订阅轮子速度 -- 赋予仿真轮子速度