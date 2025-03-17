#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <random>
#include <cmath>
#include <chrono>
#include <fstream>
#include <string>

class VirtualOdometryPublisher : public rclcpp::Node {
public:
    VirtualOdometryPublisher()
    : Node("virtual_odometry_publisher"),
      odom_publisher_(this->create_publisher<nav_msgs::msg::Odometry>("/fastlio2/lio_odom", 10)),
      imu_publisher_(this->create_publisher<sensor_msgs::msg::Imu>("/livox/imu", 10)),
      current_x_(0.0), current_y_(0.0), current_yaw_(0.0),
      current_linear_velocity_(0.0), current_angular_velocity_(0.0)
    {
        // 设置10Hz发布速率
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10Hz发布频率
            std::bind(&VirtualOdometryPublisher::publish_virtual_odometry, this)
        );

        // 订阅 /cmd_vel 来获取目标的速度命令
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&VirtualOdometryPublisher::cmd_vel_callback, this, std::placeholders::_1));

        // 创建日志文件
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
            log_file_.close();  // 确保文件在节点退出时被关闭
        }
    }

private:
    // 获取当前时间字符串
    std::string get_current_time_str() {
        auto now = std::chrono::system_clock::now();
        auto now_time_t = std::chrono::system_clock::to_time_t(now);
        std::tm tm = *std::localtime(&now_time_t);
        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        std::stringstream ss;
        ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S") << "." << std::setw(3) << std::setfill('0') << milliseconds.count();
        return ss.str();
    }

    // 处理cmd_vel消息
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // 获取目标线速度和角速度，并加噪声
        target_linear_velocity_ = msg->linear.x + generate_noise(0.00);  // 添加噪声
        target_angular_velocity_ = msg->angular.z + generate_noise(0.00);  // 添加噪声
    }

    // 计算噪声
    double generate_noise(double max_noise) {
        std::uniform_real_distribution<double> dist(-max_noise, max_noise);
        return dist(generator_);
    }

    void publish_virtual_odometry() {
        // 使用 target_linear_velocity_ 和 target_angular_velocity_ 来更新位置和朝向
        current_linear_velocity_ = target_linear_velocity_;  // 更新当前线速度
        current_angular_velocity_ = target_angular_velocity_;  // 更新当前角速度

        // 计算当前的位置和朝向（这里假设0.1秒内的运动）
        double delta_x = current_linear_velocity_ * std::cos(current_yaw_) * 0.1;
        double delta_y = current_linear_velocity_ * std::sin(current_yaw_) * 0.1;
        double delta_yaw = current_angular_velocity_ * 0.1;
    
        // 更新当前位置和朝向
        current_x_ += delta_x;
        current_y_ += delta_y;
        current_yaw_ += delta_yaw;
    
        // 确保yaw在 -π 到 π 之间
        if (current_yaw_ > M_PI) {
            current_yaw_ -= 2 * M_PI;
        } else if (current_yaw_ < -M_PI) {
            current_yaw_ += 2 * M_PI;
        }
    
        // 创建并发布Odometry消息
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "odom";
    
        // 设置位置和朝向
        odom_msg.pose.pose.position.x = current_x_;
        odom_msg.pose.pose.position.y = current_y_;
        odom_msg.pose.pose.orientation.z = std::sin(current_yaw_ / 2.0);
        odom_msg.pose.pose.orientation.w = std::cos(current_yaw_ / 2.0);
    
        // 设置线速度和角速度
        odom_msg.twist.twist.linear.x = current_linear_velocity_;
        // odom_msg.twist.twist.angular.z = current_angular_velocity_;
        odom_msg.twist.twist.angular.z = 0;
    
        odom_publisher_->publish(odom_msg);
    
        // 创建并发布IMU消息，用于模拟发布角速度
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu_link";
        // 这里我们只设置角速度，x和y可以设置为0，z为当前角速度
        imu_msg.angular_velocity.x = 0.0;
        imu_msg.angular_velocity.y = 0.0;
        imu_msg.angular_velocity.z = current_angular_velocity_;
        // 其他字段（如线加速度和姿态）可以根据需要进行设置
        imu_publisher_->publish(imu_msg);
    
        // 记录日志
        log_file_ << "Published Odometry - x: " << odom_msg.pose.pose.position.x
                  << ", y: " << odom_msg.pose.pose.position.y
                  << ", yaw: " << current_yaw_
                  << ", Linear Velocity: " << odom_msg.twist.twist.linear.x
                  << ", Angular Velocity: " << odom_msg.twist.twist.angular.z << std::endl;
    
        log_file_ << "Published IMU - Angular Velocity: ["
                  << imu_msg.angular_velocity.x << ", "
                  << imu_msg.angular_velocity.y << ", "
                  << imu_msg.angular_velocity.z << "]" << std::endl;
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    double target_linear_velocity_, target_angular_velocity_;
    double current_x_, current_y_, current_yaw_;
    double current_linear_velocity_, current_angular_velocity_;

    // 随机数生成器
    std::default_random_engine generator_;

    // 日志文件
    std::ofstream log_file_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualOdometryPublisher>());
    rclcpp::shutdown();
    return 0;
}
