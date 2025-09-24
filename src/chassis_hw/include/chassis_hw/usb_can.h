// usb_can.h
#ifndef USB_CAN_H
#define USB_CAN_H

#include <linux/can.h>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include "chassis_hw/motor_types.h" 

namespace chassis_hw
{

/**
 * @brief USB-CAN设备接口类，用于与M3508电机通信
 */
class UsbCan
{
public:
  UsbCan() = default;
  ~UsbCan();
  
  /**
   * @brief 初始化CAN设备
   * @param interface CAN接口名称(如"can0")
   * @param motor_ids 电机ID列表
   * @param thread_priority 接收线程优先级
   * @return 初始化成功返回true
   */
  bool init(const std::string& interface, const std::vector<int>& motor_ids, int thread_priority);
  
  /**
   * @brief 关闭CAN设备
   */
  void close();
  
  /**
   * @brief 获取电机数据
   * @param motor_id 电机ID
   * @return 电机数据
   */
  MotorData getMotorData(int motor_id);
  
  /**
   * @brief 发送电机命令
   * @param motor_id 电机ID
   * @param cmd 电机命令
   */
  void sendMotorCommand(int motor_id, const MotorCommand& cmd);
  
private:
  // CAN相关成员
  int sock_fd_ = -1;
  std::string interface_;
  bool is_initialized_ = false;
  
  // 线程相关
  pthread_t receiver_thread_id_;
  bool terminate_receiver_thread_ = false;
  bool receiver_thread_running_ = false;
  
  // 电机ID列表
  std::vector<int> motor_ids_;
  
  // 电机数据存储
  std::map<int, MotorData> motor_data_;
  std::mutex data_mutex_;
  
  // CAN帧处理函数
  void processCanFrame(const can_frame& frame);
  
  // 接收线程函数
  static void* receiverThread(void* arg);
  
  // M3508电机通信协议相关函数
  can_frame createM3508CommandFrame(int motor_id, const MotorCommand& cmd);
  void parseM3508FeedbackFrame(const can_frame& frame);
};

} // namespace chassis_hw

#endif // USB_CAN_H
