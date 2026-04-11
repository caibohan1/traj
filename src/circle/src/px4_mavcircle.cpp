#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/path.hpp>                 
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class MavrosOffboardTrajectory : public rclcpp::Node
{
public:
    MavrosOffboardTrajectory() : Node("mavros_offboard_trajectory")
    {
        // ================= 发布者 =================
        setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
            "/mavros/setpoint_raw/local", 10);
            
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/mavros_offboard/expected_path", 10);

        // ================= 客户端 (解锁与模式切换) =================
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

        // ================= 订阅者 =================
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", rclcpp::QoS(10).best_effort(), 
            [this](const mavros_msgs::msg::State::SharedPtr msg) {
                is_armed_ = msg->armed;
                is_offboard_ = (msg->mode == "OFFBOARD");
            });

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", rclcpp::QoS(10).best_effort(),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                current_pose_ = *msg; // 🌟 保存当前最新状态用于计算误差
                if (!has_pose_) {
                    start_x_ = msg->pose.position.x;
                    start_y_ = msg->pose.position.y;
                    start_z_ = msg->pose.position.z;
                    double q_w = msg->pose.orientation.w, q_x = msg->pose.orientation.x;
                    double q_y = msg->pose.orientation.y, q_z = msg->pose.orientation.z;
                    start_yaw_ = std::atan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z));
                    has_pose_ = true;
                    RCLCPP_INFO(this->get_logger(), "📍 Initial Pose Locked! X:%.2f, Y:%.2f, Yaw:%.2f rad", start_x_, start_y_, start_yaw_);
                }
            });

        time_step_ = 0;
        offboard_setpoint_counter_ = 0;
        init_visual_path();

        // 100Hz 主循环
        timer_ = this->create_wall_timer(
            10ms, std::bind(&MavrosOffboardTrajectory::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "🚀 MAVROS Auto-Takeoff Node Started. Waiting for Odometry...");
    }

private:
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; 
    
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    bool has_pose_ = false;
    bool is_armed_ = false;
    bool is_offboard_ = false;
    bool trajectory_started_ = false;

    geometry_msgs::msg::PoseStamped current_pose_; 
    double start_x_ = 0.0, start_y_ = 0.0, start_z_ = 0.0, start_yaw_ = 0.0;    
    uint64_t time_step_;
    uint64_t offboard_setpoint_counter_;

    nav_msgs::msg::Path expected_path_; 

    // 🌟 新增误差统计变量
    double sum_err_x_ = 0.0;
    double sum_err_y_ = 0.0;
    double sum_err_z_ = 0.0;
    uint64_t cruise_sample_count_ = 0;

    void init_visual_path() { 
        double R = 0.5; double omega_max = 1.0; 
        expected_path_.header.frame_id = "map"; 
        for (double t = 0; t <= 2.0 * M_PI / omega_max; t += 0.1) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = R * std::sin(omega_max * t);
            pose.pose.position.y = R - R * std::cos(omega_max * t); 
            pose.pose.position.z = 1.8; // MAVROS 使用 ENU 坐标系，高度为正数
            expected_path_.poses.push_back(pose);
        }
    }

    void timer_callback()
    {
        if (!has_pose_) return; 

        mavros_msgs::msg::PositionTarget setpoint_msg{};
        auto timestamp = this->get_clock()->now();
        setpoint_msg.header.stamp = timestamp;
        setpoint_msg.header.frame_id = "map";
        setpoint_msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        setpoint_msg.type_mask = 0; // 启用 位置+速度+加速度+Yaw 控制

        // ================= 状态机：自动解锁与切模式 =================
        if (!trajectory_started_) {
            // 在起飞前，设定点必须固定在起点
            setpoint_msg.position.x = start_x_;
            setpoint_msg.position.y = start_y_;
            setpoint_msg.position.z = start_z_;
            setpoint_msg.yaw = start_yaw_;
            
            setpoint_pub_->publish(setpoint_msg);

            // MAVROS 要求在切换 offboard 之前有连续的设定点流
            if (offboard_setpoint_counter_ == 100) {
                RCLCPP_INFO(this->get_logger(), "Sending Offboard & Arm commands...");
                auto mode_cmd = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                mode_cmd->custom_mode = "OFFBOARD";
                set_mode_client_->async_send_request(mode_cmd);

                auto arm_cmd = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                arm_cmd->value = true;
                arming_client_->async_send_request(arm_cmd);
            }

            // 如果还没切成功，每隔 1 秒重发一次指令（防止丢包）
            if (offboard_setpoint_counter_ > 100 && offboard_setpoint_counter_ % 100 == 0) {
                if (!is_offboard_) {
                    auto mode_cmd = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                    mode_cmd->custom_mode = "OFFBOARD";
                    set_mode_client_->async_send_request(mode_cmd);
                }
                if (!is_armed_) {
                    auto arm_cmd = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                    arm_cmd->value = true;
                    arming_client_->async_send_request(arm_cmd);
                }
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
        const double target_z = 1.8;  // MAVROS 使用 ENU 坐标系，飞行高度为正数
        
        const double T_takeoff = 5.0; 
        const double T_land    = 5.0; 
        const double circles_accel  = 1.0, circles_cruise = 2.0, circles_decel  = 1.0; 

        const double T_accel  = (circles_accel * 2.0 * M_PI * 2.0) / omega_max; 
        const double T_cruise = (circles_cruise * 2.0 * M_PI) / omega_max;       
        const double T_decel  = (circles_decel * 2.0 * M_PI * 2.0) / omega_max;    
        
        const double T_total = T_takeoff + T_accel + T_cruise + T_decel + T_land;
        const double T_wait_after_land = 3.0; 
        
        // 🌟 任务结束时打印误差总结并自动上锁
        if (t > T_total + T_wait_after_land) {
            if (cruise_sample_count_ > 0) {
                double mae_x = sum_err_x_ / cruise_sample_count_;
                double mae_y = sum_err_y_ / cruise_sample_count_;
                double mae_z = sum_err_z_ / cruise_sample_count_;
                RCLCPP_INFO(this->get_logger(), "==================================================");
                RCLCPP_INFO(this->get_logger(), "📊 Cruise Phase MAE:");
                RCLCPP_INFO(this->get_logger(), "   X: %.4f m | Y: %.4f m | Z: %.4f m", mae_x, mae_y, mae_z);
                RCLCPP_INFO(this->get_logger(), "==================================================");
            }
            RCLCPP_INFO(this->get_logger(), "🏁 Mission Accomplished. Disarming and shutting down...");
            
            auto arm_cmd = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            arm_cmd->value = false; // Disarm
            arming_client_->async_send_request(arm_cmd);
            
            rclcpp::shutdown(); 
            return; 
        }

        double pz=0, vz=0, az=0, theta=0, omega=0, alpha=0;

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

        double px = start_x_ + R * std::sin(theta);
        double py = start_y_ + R - R * std::cos(theta);
        pz = start_z_ + pz;

        // 🌟 实时误差计算
        double real_x = current_pose_.pose.position.x;
        double real_y = current_pose_.pose.position.y;
        double real_z = current_pose_.pose.position.z;

        double err_x = std::abs(px - real_x);
        double err_y = std::abs(py - real_y);
        double err_z = std::abs(pz - real_z);

        if (t >= (T_takeoff + T_accel) && t < (T_takeoff + T_accel + T_cruise)) {
            sum_err_x_ += err_x;
            sum_err_y_ += err_y;
            sum_err_z_ += err_z;
            cruise_sample_count_++;
        }

        // 装载 MAVROS 消息
        setpoint_msg.position.x = px;
        setpoint_msg.position.y = py;
        setpoint_msg.position.z = pz;
        
        setpoint_msg.velocity.x = R * std::cos(theta) * omega;
        setpoint_msg.velocity.y = R * std::sin(theta) * omega;
        setpoint_msg.velocity.z = vz;
        
        setpoint_msg.acceleration_or_force.x = R * (std::cos(theta) * alpha - std::sin(theta) * std::pow(omega, 2));
        setpoint_msg.acceleration_or_force.y = R * (std::sin(theta) * alpha + std::cos(theta) * std::pow(omega, 2));
        setpoint_msg.acceleration_or_force.z = az;
        
        setpoint_msg.yaw = start_yaw_; 

        setpoint_pub_->publish(setpoint_msg);

        expected_path_.header.stamp = timestamp;
        path_pub_->publish(expected_path_);

        // 🌟 实时日志打印
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
            "⏱️ t: %4.1f s | SP[% .2f, % .2f, % .2f] | Real[% .2f, % .2f, % .2f] | Err[%.3f, %.3f, %.3f]", 
            t, px, py, pz, real_x, real_y, real_z, err_x, err_y, err_z);

        time_step_++;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MavrosOffboardTrajectory>());
    rclcpp::shutdown();
    return 0;
}