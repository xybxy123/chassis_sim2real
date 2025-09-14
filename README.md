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