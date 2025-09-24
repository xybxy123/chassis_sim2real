#include "chassis_hw/usb_can.h"
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <ros/ros.h>
#include <pthread.h>
#include <sched.h>

namespace chassis_hw
{

UsbCan::~UsbCan()
{
  close();
}

bool UsbCan::init(const std::string& interface, const std::vector<int>& motor_ids, int thread_priority)
{
  if (is_initialized_)
  {
    ROS_WARN("USB-CAN is already initialized");
    return true;
  }
  
  interface_ = interface;
  motor_ids_ = motor_ids;
  
  // 初始化电机数据
  for (int id : motor_ids_)
  {
    motor_data_[id] = {0.0, 0.0, 0.0, 0.0, 0.0};
  }
  
  // 创建CAN socket
  sock_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock_fd_ < 0)
  {
    ROS_ERROR("Failed to create CAN socket: %s", strerror(errno));
    return false;
  }
  
  // 设置CAN接口
  ifreq ifr;
  strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  
  if (ioctl(sock_fd_, SIOCGIFINDEX, &ifr) < 0)
  {
    ROS_ERROR("Failed to get CAN interface index: %s", strerror(errno));
    ::close(sock_fd_);
    sock_fd_ = -1;
    return false;
  }
  
  // 绑定CAN接口
  sockaddr_can addr;
  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  
  if (bind(sock_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
  {
    ROS_ERROR("Failed to bind CAN socket: %s", strerror(errno));
    ::close(sock_fd_);
    sock_fd_ = -1;
    return false;
  }
  
  // 启动接收线程
  terminate_receiver_thread_ = false;
  int ret = pthread_create(&receiver_thread_id_, nullptr, receiverThread, this);
  if (ret != 0)
  {
    ROS_ERROR("Failed to create receiver thread: %s", strerror(ret));
    ::close(sock_fd_);
    sock_fd_ = -1;
    return false;
  }
  
  // 设置线程优先级
  sched_param sched;
  sched.sched_priority = thread_priority;
  if (pthread_setschedparam(receiver_thread_id_, SCHED_FIFO, &sched) != 0)
  {
    ROS_WARN("Failed to set receiver thread priority: %s", strerror(errno));
  }
  
  is_initialized_ = true;
  ROS_INFO("USB-CAN initialized successfully on interface %s", interface.c_str());
  return true;
}

void UsbCan::close()
{
  if (!is_initialized_)
    return;
  
  // 停止接收线程
  terminate_receiver_thread_ = true;
  if (receiver_thread_running_)
  {
    pthread_join(receiver_thread_id_, nullptr);
  }
  
  // 关闭socket
  if (sock_fd_ != -1)
  {
    ::close(sock_fd_);
    sock_fd_ = -1;
  }
  
  is_initialized_ = false;
  ROS_INFO("USB-CAN closed");
}

MotorData UsbCan::getMotorData(int motor_id)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  if (motor_data_.find(motor_id) == motor_data_.end())
  {
    ROS_WARN_THROTTLE(1.0, "Motor ID %d not found in motor data", motor_id);
    return {0.0, 0.0, 0.0, 0.0, 0.0};
  }
  
  return motor_data_[motor_id];
}

void UsbCan::sendMotorCommand(int motor_id, const MotorCommand& cmd)
{
  if (!is_initialized_)
  {
    ROS_WARN_THROTTLE(1.0, "USB-CAN not initialized, cannot send command");
    return;
  }
  
  // 检查电机ID是否有效
  if (std::find(motor_ids_.begin(), motor_ids_.end(), motor_id) == motor_ids_.end())
  {
    ROS_WARN_THROTTLE(1.0, "Invalid motor ID: %d", motor_id);
    return;
  }
  
  // 创建M3508命令帧
  can_frame frame = createM3508CommandFrame(motor_id, cmd);
  
  // 发送CAN帧
  ssize_t bytes_sent = write(sock_fd_, &frame, sizeof(frame));
  if (bytes_sent != sizeof(frame))
  {
    ROS_WARN_THROTTLE(1.0, "Failed to send CAN frame to motor %d: %s", motor_id, strerror(errno));
  }
}

void UsbCan::processCanFrame(const can_frame& frame)
{
  // 判断是否是M3508的反馈帧
  if (frame.can_id >= 0x201 && frame.can_id <= 0x208)
  {
    parseM3508FeedbackFrame(frame);
  }
  else
  {
    // 可以处理其他类型的CAN帧
  }
}

void* UsbCan::receiverThread(void* arg)
{
  UsbCan* usb_can = static_cast<UsbCan*>(arg);
  usb_can->receiver_thread_running_ = true;
  
  fd_set read_set;
  struct timeval timeout;
  
  while (!usb_can->terminate_receiver_thread_)
  {
    FD_ZERO(&read_set);
    FD_SET(usb_can->sock_fd_, &read_set);
    
    // 设置超时时间为100ms
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;
    
    int ret = select(usb_can->sock_fd_ + 1, &read_set, nullptr, nullptr, &timeout);
    if (ret < 0)
    {
      if (errno != EINTR)
      {
        ROS_ERROR("CAN select error: %s", strerror(errno));
        break;
      }
      continue;
    }
    else if (ret == 0)
    {
      // 超时，继续循环
      continue;
    }
    
    // 读取CAN帧
    can_frame frame;
    ssize_t bytes_read = read(usb_can->sock_fd_, &frame, sizeof(frame));
    if (bytes_read < 0)
    {
      ROS_ERROR("Failed to read CAN frame: %s", strerror(errno));
      continue;
    }
    else if (bytes_read != sizeof(frame))
    {
      ROS_WARN("Incomplete CAN frame read");
      continue;
    }
    
    // 处理接收到的CAN帧
    usb_can->processCanFrame(frame);
  }
  
  usb_can->receiver_thread_running_ = false;
  return nullptr;
}

can_frame UsbCan::createM3508CommandFrame(int motor_id, const MotorCommand& cmd)
{
  can_frame frame;
  memset(&frame, 0, sizeof(frame));
  
  frame.can_id = 0x200;
  frame.can_dlc = 8;
  
  const double MAX_CURRENT = 16384.0;  // 最大电流值
  const double MAX_TORQUE = 10.0;      // 最大力矩
  
  // 计算电流值并转换为16位整数
  int16_t current = static_cast<int16_t>(cmd.effort * MAX_CURRENT / MAX_TORQUE);
  
  switch(motor_id) {
    case 1:
      frame.data[0] = (current >> 8) & 0xFF;
      frame.data[1] = current & 0xFF;
      break;
    case 2:
      frame.data[2] = (current >> 8) & 0xFF; 
      frame.data[3] = current & 0xFF; 
      break;
    case 3:
      frame.data[4] = (current >> 8) & 0xFF;
      frame.data[5] = current & 0xFF; 
      break;
    case 4:
      frame.data[6] = (current >> 8) & 0xFF;
      frame.data[7] = current & 0xFF;
      break;
    default:

      break;
  }
  
  return frame;
}
    

void UsbCan::parseM3508FeedbackFrame(const can_frame& frame)
{
  // 从帧ID中提取电机ID
  int motor_id = frame.can_id - 0x200;
  
  // 检查电机ID是否在我们的列表中
  if (std::find(motor_ids_.begin(), motor_ids_.end(), motor_id) == motor_ids_.end())
  {
    return;
  }
  
if (frame.can_dlc == 8)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    int16_t raw_angle = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
    float current_angle = raw_angle * 2.0f * M_PI / 8191.0f;
    
    if (motor_data_[motor_id].last_angle - current_angle > M_PI) {
        motor_data_[motor_id].total_angle += 2.0f * M_PI;
    } else if (current_angle - motor_data_[motor_id].last_angle > M_PI) {
        motor_data_[motor_id].total_angle -= 2.0f * M_PI;
    }
    
    motor_data_[motor_id].angle = current_angle + motor_data_[motor_id].total_angle;
    motor_data_[motor_id].last_angle = current_angle;

    int16_t raw_velocity = (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3];
    motor_data_[motor_id].velocity = raw_velocity * 0.01f; 

    int16_t raw_torque = (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5];
    motor_data_[motor_id].effort = raw_torque * 0.1f;

    uint8_t raw_temp = frame.data[6];
    motor_data_[motor_id].temp = raw_temp; 

}

}

} // namespace chassis_hw
