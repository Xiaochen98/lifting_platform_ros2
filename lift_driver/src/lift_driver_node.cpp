#include <rclcpp/rclcpp.hpp> // ROS 2 的核心库
#include <std_msgs/msg/float32.hpp> // 用于发布高度的消息类型
#include <std_msgs/msg/string.hpp> // 用于订阅控制指令的消息类型
#include <boost/asio.hpp> // 用于串口通信的 Boost.Asio 库
#include <thread> // 用于多线程操作
#include <sstream> // 用于字符串流操作

//引入boost::asio里的类，这样使用的时候不需要带命名空间
using boost::asio::io_service; // IO 服务
using boost::asio::serial_port; // 串口
using boost::asio::serial_port_base; // 串口基础配置

// 定义一个继承自 rclcpp::Node 的类
//public 的作用
//在 C++ 继承里有三种模式：
//public 继承：基类（父类）的 public / protected 成员，在子类中保持可见。
//protected 继承：基类的 public 成员变成 protected。
//private 继承：基类的 public/protected 成员在子类里变成 private。
class LiftDriverNode : public rclcpp::Node {
public:
    // 构造函数
    //这就是 初始化列表。它告诉编译器：
    //Node("lift_driver_node") → 调用基类 rclcpp::Node 的构造函数（带一个字符串参数）。
    //io_() → 调用 io_service 的 默认构造函数。
    //serial_(io_) → 调用 serial_port 的构造函数，把 io_ 作为参数传进去。
    LiftDriverNode() : Node("lift_driver_node"), io_(), serial_(io_) {
        // --- 串口初始化 ---
        std::string port = "/dev/ttyUSB0";   // 串口设备路径，根据实际情况修改
        int baudrate = 38400; // 波特率

        try {
            // 打开串口并设置参数
            serial_.open(port);
            serial_.set_option(serial_port_base::baud_rate(baudrate)); // 设置波特率
            serial_.set_option(serial_port_base::character_size(8)); // 设置数据位
            serial_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one)); // 设置停止位
            serial_.set_option(serial_port_base::parity(serial_port_base::parity::none)); // 设置无校验位
        } catch (std::exception &e) {
            // 如果串口打开失败，打印错误信息并关闭节点
            RCLCPP_ERROR(this->get_logger(), "串口打开失败: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        // 创建一个发布器，用于发布升降平台的高度
        //在 C++ 中，this 是一个指针，它指向当前对象本身。
        //比如你写了一个类 LiftNode : public rclcpp::Node，那么在这个类的成员函数里，this 指的就是当前这个 LiftNode 对象。
        //this 是一个指针，不是对象本身。如果是对象本身调用成员函数，你会写'.'
        //但如果是通过指针调用成员函数，就要用 '->'。
        //<std_msgs::msg::Float32>尖括号里的东西是 模板参数。create_publisher 是一个 函数模板，它需要你告诉它要发布的消息类型。
        //("/lift/height", 10)小括号里的部分是 函数参数，对应 create_publisher 的参数："/lift/height"：
        //表示话题名（topic name），你这个发布器会把消息发到 /lift/height 上。
        //10：表示队列长度（QoS 的历史深度，常见用法是 rclcpp::QoS(10)）。意思是消息缓存的大小为 10 条，订阅者来不及接收时，最多保留最近的 10 条。
        height_pub_ = this->create_publisher<std_msgs::msg::Float32>("/lift/height", 10);
        // 创建一个订阅器，用于接收升降平台的控制指令
        //把类的成员函数 cmd_callback 绑定成订阅器的回调函数
        //每当 /lift/cmd 收到一条 std_msgs::msg::String 消息，就调用当前对象的 cmd_callback(msg) 来处理。
        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/lift/cmd", 10,
            std::bind(&LiftDriverNode::cmd_callback, this, std::placeholders::_1)
        );

        // 启动一个线程'read_loop'，用于读取串口数据
        read_thread_ = std::thread(&LiftDriverNode::read_loop, this);
    }

    // 析构函数
    ~LiftDriverNode() {
        // 关闭串口和线程
        if (serial_.is_open()) serial_.close(); // 如果串口打开，关闭串口
        io_.stop(); // 停止 IO 服务
        if (read_thread_.joinable()) read_thread_.join(); // 等待线程结束
    }

private:
    io_service io_; // IO 服务对象
    serial_port serial_; // 串口对象
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr height_pub_; // 高度发布器
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_; // 控制指令订阅器
    std::thread read_thread_; // 串口读取线程

    // 控制指令的回调函数
    void cmd_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string cmd = msg->data; // 获取指令数据
        std::vector<uint8_t> frame; // 用于存储发送的帧数据

        // 根据指令生成对应的帧数据
        if (cmd == "up")        frame = {0xAA, 0x03, 0x02, 0x00, 0xFF}; // 上升指令
        else if (cmd == "down") frame = {0xAA, 0x03, 0x03, 0x00, 0xFF}; // 下降指令
        else if (cmd == "stop") frame = {0xAA, 0x03, 0x04, 0x00, 0xFF}; // 停止指令
        else {
            // 如果是未知指令，打印警告信息
            RCLCPP_WARN(this->get_logger(), "未知指令: %s", cmd.c_str());
            return;
        }

        try {
            // 通过串口发送帧数据
            boost::asio::write(serial_, boost::asio::buffer(frame));
        } catch (std::exception &e) {
            // 如果发送失败，打印错误信息
            RCLCPP_ERROR(this->get_logger(), "发送失败: %s", e.what());
        }
    }

    // 串口读取线程的主循环
    void read_loop() {
        uint8_t buf[256];

        bool in_frame = false;          // 是否已看到 AA
        std::string payload;            // 累积 AA..FF 之间的字节（原样存）

        while (rclcpp::ok()) {
            try {
                size_t n = serial_.read_some(boost::asio::buffer(buf, sizeof(buf)));
                for (size_t i = 0; i < n; ++i) {
                    uint8_t b = buf[i];

                    if (!in_frame) {
                        if (b == 0xAA) {        // 帧头
                            in_frame = true;
                            payload.clear();
                        }
                        continue;
                    }

                    // 已在帧内：遇到帧尾就处理，否则继续累积
                    if (b == 0xFF) {            // 帧尾
                        // 期望载荷: 07 01 + 5字节ASCII = 共7字节
                        if (payload.size() == 7 &&
                            static_cast<uint8_t>(payload[0]) == 0x07 &&
                            static_cast<uint8_t>(payload[1]) == 0x01)
                        {
                            std::string s = payload.substr(2, 5); // 取5字节ASCII
                            try {
                                float h = std::stof(s);
                                std_msgs::msg::Float32 msg; msg.data = h;
                                height_pub_->publish(msg);
                                // 如需调试：RCLCPP_INFO(this->get_logger(), "Raw:%s  h=%.3f", s.c_str(), h);
                            } catch (const std::exception& e) {
                                RCLCPP_WARN(this->get_logger(), "Bad ASCII '%s': %s", s.c_str(), e.what());
                            }
                        }
                        // 无论是否符合期望格式，都重置，等待下一帧
                        in_frame = false;
                        payload.clear();
                    } else {
                        // 累积载荷；防御性上限，避免异常数据无限增长
                        if (payload.size() < 32) payload.push_back(static_cast<char>(b));
                        else { in_frame = false; payload.clear(); } // 超限直接丢弃重来
                    }
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                    "读取异常: %s", e.what());
            }
        }   
    }
    
};

// 主函数
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv); // 初始化 ROS 2
    auto node = std::make_shared<LiftDriverNode>(); // 创建节点对象
    rclcpp::spin(node); // 运行节点
    rclcpp::shutdown(); // 关闭 ROS 2
    return 0;
}
