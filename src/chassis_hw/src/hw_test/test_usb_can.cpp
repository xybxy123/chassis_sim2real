//rosrun chassis_hw test_usb_can _can_interface:=can0 _motor_id:=1
#include <ros/ros.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <string.h>
#include <cmath>
#include <thread>
#include <atomic>

class M3508Tester
{
public:
    M3508Tester() : running_(false), current_torque_(0.0) {}
    
    ~M3508Tester()
    {
        stop();
        if (can_socket_ >= 0)
        {
            close(can_socket_);
        }
    }
    
    bool init(const std::string& can_interface, int motor_id)
    {
        motor_id_ = motor_id;
        
        // 创建CAN socket
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0)
        {
            ROS_ERROR("Failed to create CAN socket: %s", strerror(errno));
            return false;
        }
        
        // 设置CAN接口
        struct ifreq ifr;
        strncpy(ifr.ifr_name, can_interface.c_str(), IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';
        
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
        {
            ROS_ERROR("Failed to get CAN interface index: %s", strerror(errno));
            close(can_socket_);
            can_socket_ = -1;
            return false;
        }
        
        // 绑定CAN接口
        struct sockaddr_can addr;
        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        
        if (bind(can_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0)
        {
            ROS_ERROR("Failed to bind CAN socket: %s", strerror(errno));
            close(can_socket_);
            can_socket_ = -1;
            return false;
        }
        
        ROS_INFO("M3508 tester initialized on interface %s for motor ID %d", 
                 can_interface.c_str(), motor_id);
        return true;
    }
    
    void start()
    {
        if (running_) return;
        
        running_ = true;
        // 启动接收线程
        receive_thread_ = std::thread(&M3508Tester::receiveLoop, this);
        // 启动发送线程
        send_thread_ = std::thread(&M3508Tester::sendLoop, this);
        
        ROS_INFO("Starting M3508 test...");
        ROS_INFO("Sending zero torque command. Motor should not move.");
        
        // 初始发送零力矩命令
        setTorque(0.0);
        
        // 等待一段时间后发送测试力矩
        ros::Duration(2.0).sleep();
        
        ROS_INFO("Sending test torque command. Motor should move slightly.");
        setTorque(1.0);  //  Nm
        
        // 再等待一段时间后停止
        ros::Duration(5.0).sleep();
        
        ROS_INFO("Stopping motor.");
        setTorque(0.0);
    }
    
    void stop()
    {
        running_ = false;
        if (receive_thread_.joinable())
        {
            receive_thread_.join();
        }
        if (send_thread_.joinable())
        {
            send_thread_.join();
        }
        
        // 最后发送一次停止命令
        sendTorqueCommand(0.0);
    }
    
    // 设置当前力矩值（线程安全）
    void setTorque(double torque)
    {
        current_torque_ = torque;
    }
    
private:
    // 周期性发送命令的循环
    void sendLoop()
    {
        ros::Rate rate(1000);  // 100Hz，周期10ms
        while (running_)
        {
            sendTorqueCommand(current_torque_);
            rate.sleep();
        }
    }
    
    void sendTorqueCommand(double torque_nm)
    {
        if (can_socket_ < 0) return;
        
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));
        
        // M3508命令帧ID: 0x200
        frame.can_id = 0x200;
        frame.can_dlc = 8;
        
        // 将力矩转换为电流值 
        const double max_torque = 10.0; 
        const int16_t max_current = 16384;
        
        int16_t current = static_cast<int16_t>((torque_nm / max_torque) * max_current);
        
        // 填充数据帧 
        frame.data[0] = (current >> 8) & 0xFF;
        frame.data[1] = current & 0xFF;
        
        // 发送CAN帧
        ssize_t bytes_sent = write(can_socket_, &frame, sizeof(frame));
        if (bytes_sent != sizeof(frame))
        {
            ROS_WARN("Failed to send CAN frame: %s", strerror(errno));
        }
        else
        {
            ROS_DEBUG("Sent torque command: %.2f Nm (current: %d)", torque_nm, current);
        }
    }
    
    void receiveLoop()
    {
        struct timeval timeout;
        fd_set read_set;
        
        while (running_)
        {
            FD_ZERO(&read_set);
            FD_SET(can_socket_, &read_set);
            
            timeout.tv_sec = 0;
            timeout.tv_usec = 100000;  // 100ms
            
            int ret = select(can_socket_ + 1, &read_set, NULL, NULL, &timeout);
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
            struct can_frame frame;
            ssize_t bytes_read = read(can_socket_, &frame, sizeof(frame));
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
            
            // 处理M3508反馈帧 (ID范围: 0x201-0x208)
            if (frame.can_id >= 0x201 && frame.can_id <= 0x208)
            {
                int motor_id = frame.can_id - 0x200;
                
                if (motor_id == motor_id_ && frame.can_dlc == 8)
                {
                    parseM3508Feedback(frame);
                }
            }
        }
    }
    
    void parseM3508Feedback(const can_frame& frame)
    {
        // 解析位置 (16位无符号整数，0-65535对应0-360度)
        uint16_t raw_position = (frame.data[0] << 8) | frame.data[1];
        double position = raw_position * 2.0 * M_PI / 65536.0; // 转换为弧度，0-2π
         
        // 解析速度 (RPM -> rad/s)
        int16_t raw_velocity = (frame.data[2] << 8) | frame.data[3];
        double velocity = raw_velocity * 2.0 * M_PI / 60.0; // RPM to rad/s

        // 解析转矩 (假设单位是0.01Nm，即原始值乘以0.01得到Nm)
        int16_t raw_torque = (frame.data[4] << 8) | frame.data[5];
        double torque = raw_torque * 0.01; // 转换为Nm

        // DATA[6]和DATA[7]为Null，不解析
        
        ROS_INFO("Motor %d: Pos=%.2f rad, Vel=%.2f rad/s, Torque=%.2f Nm",
                 motor_id_, position, velocity, torque);
    }
    
    int can_socket_ = -1;
    int motor_id_ = 1;
    std::atomic<bool> running_;
    std::atomic<double> current_torque_;  // 原子变量确保线程安全
    std::thread receive_thread_;
    std::thread send_thread_;  // 发送线程
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "m3508_tester");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    std::string can_interface;
    int motor_id;
    
    // 获取参数
    private_nh.param<std::string>("can_interface", can_interface, "can0");
    private_nh.param<int>("motor_id", motor_id, 1);
    
    M3508Tester tester;
    
    if (!tester.init(can_interface, motor_id))
    {
        ROS_ERROR("Failed to initialize M3508 tester");
        return 1;
    }
    
    // 运行测试
    tester.start();
    
    // 等待一段时间观察结果
    ros::Duration(5.0).sleep();
    
    tester.stop();
    ROS_INFO("M3508 test completed");
    
    return 0;
}
