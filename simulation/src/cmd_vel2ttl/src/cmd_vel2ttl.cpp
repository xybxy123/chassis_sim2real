#include "ros/ros.h"
#include <iostream>
#include <string.h>
#include <string> 
#include <math.h>
#include <stdlib.h>       
#include <unistd.h>      
#include <serial/serial.h>
#include <std_msgs/Float32.h>
// 补充Twist消息头文件
#include <geometry_msgs/Twist.h>

serial::Serial Robot_Serial;
ros::Subscriber cmd_vel_sub;

const unsigned char FRAME_HEADER = 0xAA;   // Frame header
const unsigned char FRAME_FOOTER = 0x55;   // Frame footer
unsigned char readBuff[15];  // 接收缓冲区：匹配15字节接收帧（原14字节不足，需修改）

// 计算简单的和校验（用于数据完整性校验）
unsigned char calculateChecksum(unsigned char* data, int len) {
    unsigned char checksum = 0;
    for (int i = 0; i < len; i++) {
        checksum += data[i];
    }
    return checksum;
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    float vx = msg->linear.x; 
    float vy = msg->linear.y; 
    float vyaw = msg->angular.z; 
    
    // 帧结构：帧头(1) + 数据(12) + 校验位(1) + 帧尾(1) = 15字节
    unsigned char sendBuff[15];
    
    sendBuff[0] = FRAME_HEADER;
    
    // 浮点数转字节数组
    memcpy(&sendBuff[1], &vx, sizeof(float));    // Bytes 1-4: vx
    memcpy(&sendBuff[5], &vy, sizeof(float));    // Bytes 5-8: vy
    memcpy(&sendBuff[9], &vyaw, sizeof(float));  // Bytes 9-12: vyaw
    
    // 计算校验位（校验帧头到数据部分，共13字节）
    sendBuff[13] = calculateChecksum(sendBuff, 13);
    
    // 帧尾
    sendBuff[14] = FRAME_FOOTER;
    
    if (Robot_Serial.isOpen()) {
        try {
            Robot_Serial.write(sendBuff, sizeof(sendBuff));
            ROS_DEBUG("Sent velocity command: vx=%.2f, vy=%.2f, vyaw=%.2f", vx, vy, vyaw); // 中文→英文
        } catch (serial::IOException& e) {
            ROS_ERROR("Serial write error: %s", e.what()); // 中文→英文
        }
    } else {
        ROS_WARN_THROTTLE(1.0, "Serial port not open, cannot send data"); // 中文→英文
    }
}

// 解析串口接收的数据（Parse received serial data）
void parseReceivedData(unsigned char* data, int len) {
    // 校验帧头帧尾（Check frame header and footer）
    if (data[0] != FRAME_HEADER || data[len-1] != FRAME_FOOTER) {
        ROS_WARN("Invalid frame format (header/footer mismatch)"); // 中文→英文
        return;
    }
    
    // 校验和校验（Check checksum: exclude footer, checksum is at len-2）
    unsigned char checksum = calculateChecksum(data, len-2);  
    if (data[len-2] != checksum) {
        ROS_WARN("Data checksum failed (mismatch)"); // 中文→英文
        return;
    }
    
    // 解析数据（示例：设备状态+温度，需根据实际协议调整）
    unsigned char status = data[1]; // 仅在该函数内定义，仅此处可用
    float temp;
    memcpy(&temp, &data[2], sizeof(float));
    
    // 正确用法：在变量作用域内打印日志（Print in variable's scope）
    ROS_INFO("Device Status: 0x%02X, Temperature: %.1f°C", status, temp);
}

int main(int argc, char** argv)
{    
    std::string usart_port;
    int baud_data = 115200;
    
    ros::init(argc, argv, "cmd_vel_to_serial");
    ros::NodeHandle nh_private("~");
    ros::NodeHandle node;
    
    // 读取参数（Read parameters from launch file）
    nh_private.param<std::string>("port", usart_port, "/dev/ttyUSB0");
    nh_private.param<int>("baud", baud_data, 115200);
    
    // 订阅cmd_vel话题（Subscribe to /cmd_vel topic）
    cmd_vel_sub = node.subscribe("cmd_vel", 10, cmdVelCallback);
    
    // 打印参数配置（Print parameter config: 仅保留正确的日志，删除错误的status/temp调用）
    ROS_INFO("Serial Port: %s", usart_port.c_str());
    ROS_INFO("Baud Rate: %d", baud_data);
    ROS_INFO("Waiting for /cmd_vel messages...");
    
    // 打开串口（Open serial port）
    try {
        Robot_Serial.setPort(usart_port);
        Robot_Serial.setBaudrate(baud_data);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        Robot_Serial.setTimeout(to);
        // 明确串口参数（8数据位+无校验+1停止位）
        Robot_Serial.setParity(serial::parity_none);
        Robot_Serial.setStopbits(serial::stopbits_one);
        Robot_Serial.setBytesize(serial::eightbits);
        Robot_Serial.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Failed to open serial port: " << e.what()); // 中文→英文
        return -1;
    }
    
    if (Robot_Serial.isOpen()) {
        ROS_INFO_STREAM("Serial port opened successfully"); // 中文→英文
    } else {
        ROS_ERROR_STREAM("Failed to open serial port"); // 中文→英文
        return -1;
    }
    
    // 控制主循环频率（10Hz，避免高CPU占用）
    ros::Rate loop_rate(10);
    
    // 主循环：读取并处理串口数据
    while(ros::ok()){
        // 读取15字节数据（匹配接收帧长度）
        size_t recv_len = Robot_Serial.read(readBuff, sizeof(readBuff));
        
        if (recv_len > 0) {
            if (recv_len == sizeof(readBuff)) {  // 完整帧（Complete frame）
                parseReceivedData(readBuff, recv_len);
            } else {  // 不完整帧（Incomplete frame）
                ROS_WARN("Received incomplete data, length: %zu bytes", recv_len);
                // 打印十六进制数据（便于调试）
                for (size_t i = 0; i < recv_len; i++) {
                    printf("0x%02X ", readBuff[i]);
                }
                printf("\n");
            }
        }
        
        ros::spinOnce();
        loop_rate.sleep();  // 控制循环频率
    }
    
    // 关闭串口（Close serial port）
    if (Robot_Serial.isOpen()) {
        Robot_Serial.close();
        ROS_INFO("Serial port closed"); // 中文→英文
    }
    
    return 0;
}
