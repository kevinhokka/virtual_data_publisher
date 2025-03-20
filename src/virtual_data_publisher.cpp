#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <random>
#include <cmath>
#include <chrono>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>

class VirtualOdometryPublisher : public rclcpp::Node {
public:
    VirtualOdometryPublisher()
    : Node("virtual_odometry_publisher"),
      odom_publisher_(this->create_publisher<nav_msgs::msg::Odometry>("/fastlio2/lio_odom", 100)),
      imu_publisher_(this->create_publisher<sensor_msgs::msg::Imu>("/livox/imu", 100)),
      current_x_(0.0), current_y_(0.0), current_yaw_(0.0),
      current_linear_velocity_(0.0), current_angular_velocity_(0.0),
      target_linear_velocity_(0.0), target_angular_velocity_(0.0),
      sensor_offset_x_(-0.12), sensor_offset_y_(-0.07),
      publish_odom_count_(0)  // 用于控制10Hz发布odom
    {
        // 创建100Hz的定时器（10ms触发一次）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),  // 10ms => 100Hz
            std::bind(&VirtualOdometryPublisher::timer_callback, this)
        );

        // 订阅 /cmd_vel 以获取目标速度命令
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&VirtualOdometryPublisher::cmd_vel_callback, this, std::placeholders::_1)
        );

        // 创建日志文件（确保日志文件夹存在）
        std::string log_dir = "/home/jetson/ros2_ws/src/virtual_data_publisher/logs";
        std::string log_filename = log_dir + "/" + get_current_time_str() + ".txt";
        log_file_.open(log_filename, std::ios::out);
        if (!log_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开日志文件: %s", log_filename.c_str());
            throw std::runtime_error("无法打开日志文件");
        }
    }

    ~VirtualOdometryPublisher() {
        if (log_file_.is_open()) {
            log_file_.close();  // 节点退出时关闭日志文件
        }
    }

private:
    // 获取当前时间字符串，用于日志文件命名
    std::string get_current_time_str() {
        auto now = std::chrono::system_clock::now();
        auto now_time_t = std::chrono::system_clock::to_time_t(now);
        std::tm tm = *std::localtime(&now_time_t);
        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        std::stringstream ss;
        ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S") << "."
           << std::setw(3) << std::setfill('0') << milliseconds.count();
        return ss.str();
    }

    // /cmd_vel 订阅回调，更新目标速度（可添加噪声）
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        target_linear_velocity_ = msg->linear.x + generate_noise(0.00);
        target_angular_velocity_ = msg->angular.z + generate_noise(0.00);
    }

    // 生成噪声函数，max_noise为噪声幅值
    double generate_noise(double max_noise) {
        std::uniform_real_distribution<double> dist(-max_noise, max_noise);
        return dist(generator_);
    }

    // 定时器回调函数：每 10ms 触发一次 => 100Hz
    void timer_callback() {
        // 1) 更新运动学状态
        update_robot_state(0.01);  // dt = 0.01秒(10ms)

        // 2) 发布 IMU (100Hz)
        publish_imu();

        // 3) 控制 Odom 发布频率（只在10次循环时发布一次 => 10Hz）
        publish_odom_count_++;
        if (publish_odom_count_ >= 10) {
            publish_odom_count_ = 0;
            publish_odometry();
        }
    }

    // 更新机器人的位置、速度等
    void update_robot_state(double dt) {
        // 模拟速度延迟（使用一阶低通滤波器）
        double tau_linear = 0.1;    // 线速度延迟时间常数
        double tau_angular = 0.4;   // 角速度延迟时间常数

        current_linear_velocity_ +=
            (target_linear_velocity_ - current_linear_velocity_) * (dt / tau_linear);
        current_angular_velocity_ +=
            (target_angular_velocity_ - current_angular_velocity_) * (dt / tau_angular);

        // 添加噪声
        double noise_linear = generate_noise(0.0);   // 线速度噪声
        double noise_angular = generate_noise(0.02);  // 角速度噪声
        current_linear_velocity_ += noise_linear;
        current_angular_velocity_ += noise_angular;

        // 根据实际速度计算机器人中心位置更新
        double delta_x = current_linear_velocity_ * std::cos(current_yaw_) * dt;
        double delta_y = current_linear_velocity_ * std::sin(current_yaw_) * dt;
        double delta_yaw = current_angular_velocity_ * dt;
        current_x_ += delta_x;
        current_y_ += delta_y;
        current_yaw_ += delta_yaw;

        // 保证 yaw 在 [-π, π] 范围内
        if (current_yaw_ > M_PI) {
            current_yaw_ -= 2 * M_PI;
        } else if (current_yaw_ < -M_PI) {
            current_yaw_ += 2 * M_PI;
        }

        // 日志中可以先记录“真实”机器人中心状态
        log_file_ << "[Update] True Position - x: " << current_x_
                  << ", y: " << current_y_ << ", Yaw: " << current_yaw_ << std::endl;
    }

    // 发布IMU消息（100Hz）
    void publish_imu() {
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu_link";

        // 只模拟角速度
        imu_msg.angular_velocity.x = 0.0;
        imu_msg.angular_velocity.y = 0.0;
        imu_msg.angular_velocity.z = current_angular_velocity_;

        imu_publisher_->publish(imu_msg);

        // 日志记录
        log_file_ << "[IMU] Angular Velocity: " << current_angular_velocity_ << std::endl;
    }

    // 发布Odometry消息（10Hz）
    void publish_odometry() {
        // 根据传感器偏差，计算传感器的测量位置
        double sensor_x = current_x_
            + std::cos(current_yaw_) * sensor_offset_x_
            - std::sin(current_yaw_) * sensor_offset_y_
            + generate_noise(0.05);

        double sensor_y = current_y_
            + std::sin(current_yaw_) * sensor_offset_x_
            + std::cos(current_yaw_) * sensor_offset_y_
            + generate_noise(0.05);

        // 根据传感器偏差计算线速度测量值
        // 公式： v_sensor = v_center - ω * (sensor_offset_y)
        double sensor_linear_velocity =
            current_linear_velocity_ - current_angular_velocity_ * sensor_offset_y_;

        // 构造并发布 Odometry 消息（发布的是传感器测量值）
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.pose.pose.position.x = sensor_x;
        odom_msg.pose.pose.position.y = sensor_y;
        // 将 yaw 转换为四元数（2D情况下只需 z 与 w 分量）
        odom_msg.pose.pose.orientation.z = std::sin(current_yaw_ / 2.0);
        odom_msg.pose.pose.orientation.w = std::cos(current_yaw_ / 2.0);

        // 发布修正后的线速度与角速度 (此处角速度给0，参见原逻辑)
        odom_msg.twist.twist.linear.x = sensor_linear_velocity;
        odom_msg.twist.twist.angular.z = 0.0;
        odom_publisher_->publish(odom_msg);

        // 日志记录
        log_file_ << "[Odom] Published Sensor Position - x: " << sensor_x
                  << ", y: " << sensor_y << std::endl;
        log_file_ << "[Odom] Published Sensor Linear Vel: " << sensor_linear_velocity
                  << ", Angular Vel (z=0): 0" << std::endl;
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 机器人及传感器相关变量
    double target_linear_velocity_, target_angular_velocity_;
    double current_x_, current_y_, current_yaw_;
    double current_linear_velocity_, current_angular_velocity_;
    const double sensor_offset_x_;
    const double sensor_offset_y_;

    // 用于产生噪声
    std::default_random_engine generator_;

    // 计数器，用于控制 Odom 的发布频率(10次/100ms循环 -> 10Hz)
    int publish_odom_count_;

    // 日志文件
    std::ofstream log_file_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualOdometryPublisher>());
    rclcpp::shutdown();
    return 0;
}
