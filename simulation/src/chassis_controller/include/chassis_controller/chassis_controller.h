#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace chassis_controller{ 

class ChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;
  

private:
  hardware_interface::JointHandle front_left_joint_, front_right_joint_, 
                                 back_left_joint_, back_right_joint_; 
  std::string front_left_joint_name_, front_right_joint_name_, 
              back_left_joint_name_, back_right_joint_name_;
  
  double wheel_radius_ = 0.07625; 
  double wheel_base_ = 0.4;
  double wheel_track_ = 0.4; 
  
  ros::Subscriber vel_sub_;
  geometry_msgs::Twist vel_msg_;
  void velCallback(const geometry_msgs::Twist::ConstPtr& msg); 
};

}//namespace
