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
      x_(0.0), y_(0.0), yaw_(0.0),
      vx_(0.0), vy_(0.0), omega_(0.0),
      target_vx_(0.0), target_vy_(0.0), target_omega_(0.0),
      sensor_offset_x_(-0.12), sensor_offset_y_(-0.07),
      publish_odom_count_(0)
    {
        // 定时器：100Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&VirtualOdometryPublisher::timer_callback, this)
        );

        // 订阅 /cmd_vel
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&VirtualOdometryPublisher::cmd_vel_callback, this, std::placeholders::_1)
        );

        // 创建日志文件
        std::string log_dir = "/home/jetson/ros2_ws/src/virtual_data_publisher/logs";
        std::string log_filename = log_dir + "/" + get_current_time_str() + ".txt";
        log_file_.open(log_filename, std::ios::out);
        if (!log_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开日志文件: %s", log_filename.c_str());
            throw std::runtime_error("无法打开日志文件");
        }

        // 随机数种子（用于噪声、随机扰动等）
        generator_.seed(std::random_device{}());
    }

    ~VirtualOdometryPublisher() {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

private:
    // 获取当前时间字符串
    std::string get_current_time_str() {
        auto now = std::chrono::system_clock::now();
        auto now_time_t = std::chrono::system_clock::to_time_t(now);
        std::tm tm = *std::localtime(&now_time_t);
        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()
        ) % 1000;
        std::stringstream ss;
        ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S") << "."
           << std::setw(3) << std::setfill('0') << milliseconds.count();
        return ss.str();
    }

    // 订阅 /cmd_vel 回调
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // 只用 linear.x / angular.z（可扩展为 msg->linear.y）
        target_vx_ = msg->linear.x;  // + generate_noise(...) if needed
        target_vy_ = 0.0;
        target_omega_ = msg->angular.z;
    }

    double generate_noise(double max_noise) {
        std::uniform_real_distribution<double> dist(-max_noise, max_noise);
        return dist(generator_);
    }

    // 100Hz定时器
    void timer_callback() {
        double dt = 0.01; // 100Hz

        // 1) 更新机器人动力学
        update_robot_state(dt);

        // 2) 发布IMU
        publish_imu();

        // 3) 10Hz发布Odom
        publish_odom_count_++;
        if (publish_odom_count_ >= 10) {
            publish_odom_count_ = 0;
            publish_odometry();
        }
    }

    void update_robot_state(double dt) {
        //
        // ---- 物理/模型参数设置 (示例) ----
        //

        // 1. 机器人整体
        double m = 25.0;    // 质量 [kg]
        double I_z = 4.0;   // 绕Z轴转动惯量 [kg*m^2] (改小一点,看你实际形状)

        // 2. PID控制增益（模拟下位机PID对速度/角速度的追踪）
        //    根据你下位机PID(如 3.2832 / 0 / ? )可灵活调整
        //    数值可根据观测到的仿真效果再调
        double Kp_lin = 300.0;
        double Kp_rot = 80.0;

        // 3. 粘性+库仑摩擦参数
        //    - b_lin, b_rot: 速度越大，阻力越大
        //    - f_coulomb_lin, f_coulomb_rot: 库仑摩擦(运动中几乎恒定的干摩擦)，
        //      速度接近0时会“卡住”或需要一定力才能推动。
        double b_lin = 12.0;           // 线性粘性摩擦系数
        double b_rot = 10.0;           // 角度粘性摩擦系数
        double f_coulomb_lin = 5.0;    // 库仑摩擦(线速度)
        // double f_coulomb_rot = 3.0;    // 库仑摩擦(角速度)
        double f_coulomb_rot = 20.0;    // 库仑摩擦(角速度)
        //
        // --- 计算控制力 / 力矩 (类似下位机根据速度误差输出扭矩) ---
        //
        double vx_error = (target_vx_ - vx_);
        double vy_error = (target_vy_ - vy_);
        double omega_error = (target_omega_ - omega_);

        double Fx_control = Kp_lin * vx_error;
        double Fy_control = Kp_lin * vy_error;
        double Tau_control = Kp_rot * omega_error;

        // 先计算粘性阻力
        double Fx_fric_visc = -b_lin * vx_;
        double Fy_fric_visc = -b_lin * vy_;
        double Tau_fric_visc = -b_rot * omega_;

        // 再计算库仑摩擦：
        //   sign(v) = +1(正速度), -1(负速度), 0(速度极小)
        //   速度接近0时，可以加简单阈值处理 => 如果控制力Fx_control还不足以克服库仑摩擦，则速度=0
        //   这里先做个直接减去 sign * f_coulomb
        auto sign_func = [](double x){
            if (x > 1e-3) return 1.0;
            else if (x < -1e-3) return -1.0;
            else return 0.0; // 速度太小就近似看做 0
        };

        double Fx_fric_coulomb = -f_coulomb_lin * sign_func(vx_);
        double Fy_fric_coulomb = -f_coulomb_lin * sign_func(vy_);
        double Tau_fric_coulomb = -f_coulomb_rot * sign_func(omega_);

        // 合力 = 控制力 + 粘性阻力 + 库仑摩擦
        // 注意：若速度极小且合力不足以克服静摩擦，可把速度拉回0。这里只是简化实现。
        double Fx = Fx_control + Fx_fric_visc + Fx_fric_coulomb;
        double Fy = Fy_control + Fy_fric_visc + Fy_fric_coulomb;
        double Tau_z = Tau_control + Tau_fric_visc + Tau_fric_coulomb;

        // --- 计算加速度 ---
        double ax = Fx / m;
        double ay = Fy / m;
        double alpha = Tau_z / I_z;

        // --- 判断如果速度极小且控制力+粘性力不足克服库仑摩擦，就锁死在0 ---
        //     这段可选，如果想要“静摩擦锁止”效果更明显，可以启用。
        double vel_thresh = 0.01; // 判断“极小速度”的阈值
        if (std::fabs(vx_) < vel_thresh) {
            double net_force_x = Fx_control + Fx_fric_visc;
            // 若 net_force_x 的绝对值比 f_coulomb_lin 小，则不动
            if (std::fabs(net_force_x) < f_coulomb_lin) {
                // 把ax置0，并且速度=0
                ax = 0.0;
                vx_ = 0.0;
            }
        }
        if (std::fabs(omega_) < vel_thresh) {
            double net_torque_z = Tau_control + Tau_fric_visc;
            if (std::fabs(net_torque_z) < f_coulomb_rot) {
                alpha = 0.0;
                omega_ = 0.0;
            }
        }

        // --- 更新速度(离散积分) ---
        vx_ += ax * dt;
        vy_ += ay * dt;
        omega_ += alpha * dt;

        // （可选）模拟地面随机颠簸，对速度加一点随机抖动
        // vx_ += generate_noise(0.01);
        // omega_ += generate_noise(0.005);

        // --- 将机体系速度转换到世界系，并更新位姿 ---
        double cos_yaw = std::cos(yaw_);
        double sin_yaw = std::sin(yaw_);

        double global_vx = vx_ * cos_yaw - vy_ * sin_yaw;
        double global_vy = vx_ * sin_yaw + vy_ * cos_yaw;

        x_ += global_vx * dt;
        y_ += global_vy * dt;
        yaw_ += omega_ * dt;

        // yaw 限制在 [-pi, pi]
        if (yaw_ > M_PI) yaw_ -= 2.0 * M_PI;
        else if (yaw_ < -M_PI) yaw_ += 2.0 * M_PI;
    }

    // 发布 IMU
    void publish_imu() {
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu_link";

        // 简化只模拟角速度
        imu_msg.angular_velocity.x = 0.0;
        imu_msg.angular_velocity.y = 0.0;
        imu_msg.angular_velocity.z = omega_;

        imu_publisher_->publish(imu_msg);

        // 记录日志
        log_file_ << "[IMU] Angular Velocity: " << omega_ << std::endl;
    }

    // 发布 Odom
    void publish_odometry() {
        double sensor_x = x_
            + std::cos(yaw_) * sensor_offset_x_
            - std::sin(yaw_) * sensor_offset_y_
            + generate_noise(0.01);

        double sensor_y = y_
            + std::sin(yaw_) * sensor_offset_x_
            + std::cos(yaw_) * sensor_offset_y_
            + generate_noise(0.01);

        // 这里仅模拟 x 方向速度，若需要 y，可以一起发布
        double measured_vx = vx_ - omega_ * sensor_offset_y_;
        double measured_vy = 0.0;

        // 构造Odometry
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "odom";

        odom_msg.pose.pose.position.x = sensor_x;
        odom_msg.pose.pose.position.y = sensor_y;
        odom_msg.pose.pose.orientation.z = std::sin(yaw_ / 2.0);
        odom_msg.pose.pose.orientation.w = std::cos(yaw_ / 2.0);

        odom_msg.twist.twist.linear.x = measured_vx;
        odom_msg.twist.twist.linear.y = measured_vy;
        // 看需求决定要不要发布角速度
        odom_msg.twist.twist.angular.z = 0.0;

        odom_publisher_->publish(odom_msg);

        // 日志
        log_file_ << "[Odom] sensor_x: " << sensor_x << ", sensor_y: " << sensor_y << std::endl;
        log_file_ << "[Odom] vx: " << measured_vx << ", vy: " << measured_vy << std::endl;
    }

    // ------ 成员变量 ------
private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 机器人状态
    double x_, y_, yaw_;
    double vx_, vy_, omega_;       
    double target_vx_, target_vy_, target_omega_;

    // 传感器位置偏移
    const double sensor_offset_x_;
    const double sensor_offset_y_;

    int publish_odom_count_;

    // 随机数生成器
    std::default_random_engine generator_;

    // 日志文件
    std::ofstream log_file_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualOdometryPublisher>());
    rclcpp::shutdown();
    return 0;
}
