#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char** argv) {
    // 1. 初始化 ROS 节点
    ros::init(argc, argv, "chassis_plugin_loader");
    ros::NodeHandle nh;
    ros::NodeHandle robot_hw_nh("");

    // 2. 通过 pluginlib 加载 ChassisHW 插件
    pluginlib::ClassLoader<hardware_interface::RobotHW> loader(
        "hardware_interface", "hardware_interface::RobotHW");
    boost::shared_ptr<hardware_interface::RobotHW> hw;

    try {
        hw = loader.createInstance("chassis_hw/ChassisHW");
        ROS_INFO("Successfully loaded ChassisHW plugin!");
    } catch (pluginlib::PluginlibException& ex) {
        ROS_ERROR("Failed to load plugin: %s", ex.what());
        return 1;
    }

    // 3. 初始化硬件接口（调用 ChassisHW::init()）
    if (!hw->init(nh, robot_hw_nh)) {
        ROS_ERROR("Failed to initialize hardware interface!");
        return 1;
    }

    // 4. 初始化控制器管理器，绑定硬件接口
    controller_manager::ControllerManager cm(hw.get(), nh);

    // 5. 设置控制循环频率
    ros::Rate rate(100.0);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 6. 控制循环：读取硬件状态 → 控制器计算命令 → 写入硬件
    while (ros::ok()) {
        hw->read(ros::Time::now(), rate.expectedCycleTime()); 
        cm.update(ros::Time::now(), rate.expectedCycleTime()); 
        hw->write(ros::Time::now(), rate.expectedCycleTime()); 
        rate.sleep();
    }

    spinner.stop();
    return 0;
}
