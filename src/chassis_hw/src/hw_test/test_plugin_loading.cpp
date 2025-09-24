#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <hardware_interface/robot_hw.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_plugin_loading");
  
  pluginlib::ClassLoader<hardware_interface::RobotHW> loader(
    "hardware_interface", "hardware_interface::RobotHW");
  
  try
  {
    boost::shared_ptr<hardware_interface::RobotHW> hw = 
      loader.createInstance("chassis_hw/ChassisHW");
    ROS_INFO("Successfully loaded chassis_hw plugin!");
    return 0;
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("Failed to load plugin: %s", ex.what());
    return 1;
  }
}