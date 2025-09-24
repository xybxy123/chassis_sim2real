# Simulation
## Install

```bash
#克隆仓库
git clone https://github.com/xybxy123/simulation.git

#进入项目目录
cd simulation

#编译项目
catkin build

#加载环境变量
source devel/setup.bash

#启动底盘控制节点
roslaunch chassis_controller chassis_control.launch
```

## 常见问题（Common Issues）
编译时报错：[build] Error: Unable to find source space /home/xxx/src``
若出现上述错误，可尝试以下解决方法：

```bash
#返回用户主目录
cd 

#删除catkin工具的缓存文件
rm -rf .catkin_tools
```


清除缓存后，重新回到项目目录执行编译步骤即可：

```bash
cd path/to/simulation
catkin build
```
# chassis_sim2real

基于ROS的底盘仿真与实物控制集成包，支持通过usb_can驱动m3508电机并提供标准化硬件接口。

## 功能概述

- 通过usb_can驱动m3508无刷电机
- 提供ROS硬件接口，支持与ros_control等框架集成
- 支持仿真与实物控制模式切换

## 硬件准备

1. m3508无刷电机及配套电调
2. usb_can适配器


## 安装usb_can硬件驱动

详细安装步骤请参考：`usb_can_hw_drive/操作说明.txt`

