#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp> // 🌟 新增：Offboard 心跳包
#include <px4_msgs/msg/vehicle_command.hpp>       // 🌟 新增：指令发送
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <nav_msgs/msg/path.hpp>                 
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class Px4OffboardTrajectory : public rclcpp::Node
{
public:
    Px4OffboardTrajectory() : Node("px4_offboard_trajectory")
    {
        // ================= 发布者 =================
        setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
            
        // 🌟 发布 Offboard 控制模式（心跳包）
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
            
        // 🌟 发布车辆指令（解锁、切模式）
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/px4_offboard/expected_path", 10);

        // ================= 订阅者 =================
        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", rclcpp::QoS(10).best_effort(), 
            [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
                is_armed_ = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
                is_offboard_ = (msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
            });

        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(),
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                if (!has_odom_) {
                    // 🌟 只在第一次收到里程计时锁定原点
                    start_x_ = msg->position[0];
                    start_y_ = msg->position[1];
                    start_z_ = msg->position[2];
                    double q_w = msg->q[0], q_x = msg->q[1], q_y = msg->q[2], q_z = msg->q[3];
                    start_yaw_ = std::atan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z));
                    has_odom_ = true;
                    RCLCPP_INFO(this->get_logger(), "📍 Initial Pose Locked! X:%.2f, Y:%.2f, Yaw:%.2f rad", start_x_, start_y_, start_yaw_);
                }
            });

        time_step_ = 0;
        offboard_setpoint_counter_ = 0;
        init_visual_path();

        // 100Hz 主循环
        timer_ = this->create_wall_timer(
            10ms, std::bind(&Px4OffboardTrajectory::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "🚀 Auto-Takeoff Node Started. Connecting to PX4...");
    }

private:
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; 
    
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    bool has_odom_ = false;
    bool is_armed_ = false;
    bool is_offboard_ = false;
    bool trajectory_started_ = false;

    double start_x_ = 0.0, start_y_ = 0.0, start_z_ = 0.0, start_yaw_ = 0.0;    
    uint64_t time_step_;
    uint64_t offboard_setpoint_counter_;

    nav_msgs::msg::Path expected_path_; 

    void init_visual_path() { /* 同前，略（原封不动保留你的 R=0.5 生成逻辑） */ 
        double R = 0.5; double omega_max = 1.0; 
        expected_path_.header.frame_id = "map"; 
        for (double t = 0; t <= 2.0 * M_PI / omega_max; t += 0.1) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = R * std::sin(omega_max * t);
            pose.pose.position.y = R - R * std::cos(omega_max * t); 
            pose.pose.position.z = -1.8; 
            expected_path_.poses.push_back(pose);
        }
    }

    // 🌟 发送 Offboard 控制模式（声明我们要控制什么）
    void publish_offboard_control_mode() {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = true;
        msg.acceleration = true;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_pub_->publish(msg);
    }

    // 🌟 发送底层系统指令
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_pub_->publish(msg);
    }

    void timer_callback()
    {
        if (!has_odom_) return; // 没收到里程计前什么都不做

        // 1. 必须始终发布 Offboard 心跳包
        publish_offboard_control_mode();

        px4_msgs::msg::TrajectorySetpoint setpoint_msg{};
        auto timestamp = this->get_clock()->now();
        setpoint_msg.timestamp = timestamp.nanoseconds() / 1000;

        // ================= 状态机：自动解锁与切模式 =================
        if (!trajectory_started_) {
            // 在起飞前，设定点必须固定在起点
            setpoint_msg.position = {(float)start_x_, (float)start_y_, (float)start_z_};
            setpoint_msg.yaw = (float)start_yaw_;
            setpoint_msg.velocity = {0.0, 0.0, 0.0};
            setpoint_msg.acceleration = {0.0, 0.0, 0.0};
            setpoint_pub_->publish(setpoint_msg);

            // PX4 要求在切换 offboard 之前至少有 1 秒的设定点流（100次循环）
            if (offboard_setpoint_counter_ == 100) {
                RCLCPP_INFO(this->get_logger(), "Sending Offboard & Arm commands...");
                // 发送切 Offboard 指令 (1 代表自定义模式, 6 代表 Offboard)
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                // 发送解锁指令 (1.0 代表 Arm)
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
            }

            // 如果还没切成功，每隔 1 秒重发一次指令（防止丢包）
            if (offboard_setpoint_counter_ > 100 && offboard_setpoint_counter_ % 100 == 0) {
                if (!is_offboard_) publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                if (!is_armed_) publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
            }

            if (is_armed_ && is_offboard_) {
                RCLCPP_INFO(this->get_logger(), "✅ Offboard & Armed! Executing Trajectory...");
                trajectory_started_ = true;
            }

            offboard_setpoint_counter_++;
            return; 
        }
                
        // ================= 正式进入轨迹执行 =================
        double t = static_cast<double>(time_step_) * 0.01; 
        
        const double R = 0.5;        
        const double omega_max = 1.0;   
        const double target_z = -1.8; 
        
        const double T_takeoff = 5.0; 
        const double T_land    = 5.0; 
        const double circles_accel  = 1.0, circles_cruise = 2.0, circles_decel  = 1.0; 

        const double T_accel  = (circles_accel * 2.0 * M_PI * 2.0) / omega_max; 
        const double T_cruise = (circles_cruise * 2.0 * M_PI) / omega_max;       
        const double T_decel  = (circles_decel * 2.0 * M_PI * 2.0) / omega_max;    
        
        const double T_total = T_takeoff + T_accel + T_cruise + T_decel + T_land;
        const double T_wait_after_land = 3.0; 
        
        if (t > T_total + T_wait_after_land) {
            RCLCPP_INFO(this->get_logger(), "🏁 Mission Accomplished. Disarming and shutting down...");
            // 落地后自动上锁
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
            rclcpp::shutdown(); 
            return; 
        }

        double pz=0, vz=0, az=0, theta=0, omega=0, alpha=0;

        // 五段式运动学状态机（逻辑与之前完全一致，保留多项式平滑起降）
        if (t < T_takeoff) {
            double tau = t / T_takeoff;
            double tau2 = tau * tau; double tau3 = tau2 * tau;
            double tau4 = tau3 * tau; double tau5 = tau4 * tau;
            double S = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
            double dS = (1.0 / T_takeoff) * (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4);
            double ddS = (1.0 / std::pow(T_takeoff, 2)) * (60.0 * tau - 180.0 * tau2 + 120.0 * tau3);

            pz = target_z * S; vz = target_z * dS; az = target_z * ddS; 
        } 
        else if (t < T_takeoff + T_accel) {
            double tau = (t - T_takeoff) / T_accel;
            double tau2 = tau * tau, tau3 = tau2 * tau, tau4 = tau3 * tau, tau5 = tau4 * tau, tau6 = tau5 * tau;
            double S = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
            double dS = (1.0 / T_accel) * (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4);

            pz = target_z; omega = omega_max * S; alpha = omega_max * dS; 
            theta = omega_max * T_accel * (2.5 * tau4 - 3.0 * tau5 + tau6); 
        } 
        else if (t < T_takeoff + T_accel + T_cruise) {
            pz = target_z; omega = omega_max; 
            theta = (circles_accel * 2.0 * M_PI) + (omega_max * (t - T_takeoff - T_accel));
        } 
        else if (t < T_takeoff + T_accel + T_cruise + T_decel) {
            double tau = (t - T_takeoff - T_accel - T_cruise) / T_decel;
            double tau2 = tau * tau, tau3 = tau2 * tau, tau4 = tau3 * tau, tau5 = tau4 * tau, tau6 = tau5 * tau;
            double S = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
            double dS = (1.0 / T_decel) * (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4);

            pz = target_z; omega = omega_max * (1.0 - S); alpha = -omega_max * dS; 
            theta = (circles_accel + circles_cruise) * 2.0 * M_PI + (omega_max * T_decel * (tau - (2.5 * tau4 - 3.0 * tau5 + tau6))); 
        } 
        else if (t < T_total) {
            double tau = (t - T_takeoff - T_accel - T_cruise - T_decel) / T_land;
            double tau2 = tau * tau, tau3 = tau2 * tau, tau4 = tau3 * tau, tau5 = tau4 * tau;
            double S = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
            double dS = (1.0 / T_land) * (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4); 
            double ddS = (1.0 / std::pow(T_land, 2)) * (60.0 * tau - 180.0 * tau2 + 120.0 * tau3);

            pz = target_z * (1.0 - S); vz = -target_z * dS; az = -target_z * ddS; 
            theta = (circles_accel + circles_cruise + circles_decel) * 2.0 * M_PI; 
        }
        else {
            pz = 0; theta = (circles_accel + circles_cruise + circles_decel) * 2.0 * M_PI;
        }

        // ================= 运动学映射 =================
        double px = start_x_ + R * std::sin(theta);
        double py = start_y_ + R - R * std::cos(theta);
        pz = start_z_ + pz;
        
        setpoint_msg.position = {(float)px, (float)py, (float)pz};
        setpoint_msg.velocity = {(float)(R * std::cos(theta) * omega), (float)(R * std::sin(theta) * omega), (float)vz};
        setpoint_msg.acceleration = {(float)(R * (std::cos(theta) * alpha - std::sin(theta) * std::pow(omega, 2))), 
                                     (float)(R * (std::sin(theta) * alpha + std::cos(theta) * std::pow(omega, 2))), (float)az};
        setpoint_msg.yaw = (float)start_yaw_; 

        setpoint_pub_->publish(setpoint_msg);

        expected_path_.header.stamp = timestamp;
        path_pub_->publish(expected_path_);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "✈️ Flying -> Z: %.2f m | Mode Time: %.1f s", pz, t);

        time_step_++;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4OffboardTrajectory>());
    rclcpp::shutdown();
    return 0;
}