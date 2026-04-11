#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/path.hpp>                 
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>   
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class MavrosAutoFigure8 : public rclcpp::Node
{
public:
    MavrosAutoFigure8() : Node("mavros_auto_figure8")
    {
        // ================= 发布者 =================
        setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
            "/mavros/setpoint_raw/local", 10);
            
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/mavros_offboard/expected_path", 10);

        error_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/mavros_offboard/position_error", 10);

        // ================= 服务客户端 =================
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
                
                current_pose_ = *msg; 

                if (!has_pose_) {
                    start_x_ = msg->pose.position.x;
                    start_y_ = msg->pose.position.y;
                    start_z_ = msg->pose.position.z;
                    double q_w = msg->pose.orientation.w, q_x = msg->pose.orientation.x;
                    double q_y = msg->pose.orientation.y, q_z = msg->pose.orientation.z;
                    start_yaw_ = std::atan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z));
                    has_pose_ = true;
                    
                    RCLCPP_INFO(this->get_logger(), "📍 Initial Pose Locked! X:%.2f, Y:%.2f, Yaw:%.2f rad", start_x_, start_y_, start_yaw_);
                    
                    init_visual_path();
                }
            });

        time_step_ = 0;
        offboard_setpoint_counter_ = 0;

        timer_ = this->create_wall_timer(
            10ms, std::bind(&MavrosAutoFigure8::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "🚀 Auto-Takeoff Figure-8 Node (MAVROS) Started. Connecting...");
    }

private:
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; 
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr error_pub_; 
    
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::PoseStamped current_pose_;

    bool has_pose_ = false;
    bool is_armed_ = false;
    bool is_offboard_ = false;
    bool trajectory_started_ = false;

    double start_x_ = 0.0, start_y_ = 0.0, start_z_ = 0.0, start_yaw_ = 0.0;    
    uint64_t time_step_;
    uint64_t offboard_setpoint_counter_;

    nav_msgs::msg::Path expected_path_; 

    // 🌟 统计变量
    double sum_error_x_ = 0.0;
    double sum_error_y_ = 0.0;
    double sum_error_z_ = 0.0;
    uint64_t cruise_sample_count_ = 0;

    void init_visual_path()
    {
        double A = 2.0;  
        double B = 1.0;  
        double omega_max = 1.8; 
        double target_z = 1.8; // MAVROS (ENU) 向上为正
        
        expected_path_.header.frame_id = "map"; 
        expected_path_.poses.clear();
        
        double T = 2.0 * M_PI / omega_max; 
        for (double t = 0; t <= T; t += 0.05) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = start_x_ + A * std::sin(omega_max * t);
            pose.pose.position.y = start_y_ + B * std::sin(2.0 * omega_max * t);
            pose.pose.position.z = start_z_ + target_z; 
            expected_path_.poses.push_back(pose);
        }
    }

    void timer_callback()
    {
        if (!has_pose_) return;

        mavros_msgs::msg::PositionTarget msg{};
        auto timestamp = this->get_clock()->now();
        msg.header.stamp = timestamp;
        msg.header.frame_id = "map";
        msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED; // 通常MAVROS默认这个框架
        msg.type_mask = 0; // 启用位置、速度、加速度和 Yaw 控制

        // ================= 状态机：自动解锁与切模式 =================
        if (!trajectory_started_) {
            msg.position.x = start_x_;
            msg.position.y = start_y_;
            msg.position.z = start_z_;
            msg.yaw = start_yaw_;
            
            setpoint_pub_->publish(msg);

            geometry_msgs::msg::PointStamped error_msg;
            error_msg.header.stamp = timestamp;
            error_msg.header.frame_id = "map";
            error_msg.point.x = 0.0;
            error_msg.point.y = 0.0;
            error_msg.point.z = 0.0;
            error_pub_->publish(error_msg);

            if (offboard_setpoint_counter_ == 100) {
                RCLCPP_INFO(this->get_logger(), "Sending Offboard & Arm commands...");
                auto mode_cmd = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                mode_cmd->custom_mode = "OFFBOARD";
                set_mode_client_->async_send_request(mode_cmd);

                auto arm_cmd = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                arm_cmd->value = true;
                arming_client_->async_send_request(arm_cmd);
            }

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
                RCLCPP_INFO(this->get_logger(), "✅ Offboard & Armed! Executing Figure-8 Trajectory...");
                trajectory_started_ = true;
            }

            offboard_setpoint_counter_++;
            return; 
        }

        // ================= 正式进入轨迹执行 =================
        double t = static_cast<double>(time_step_) * 0.01; 
        
        const double A = 2.0;        
        const double B = 1.0;        
        const double omega_max = 1.8;   
        const double target_z = 1.8; // ENU  
        
        const double T_takeoff = 5.0; 
        const double T_land    = 5.0; 

        const double circles_accel  = 1.0; 
        const double circles_cruise = 2.0; 
        const double circles_decel  = 1.0; 

        const double T_accel  = (circles_accel * 2.0 * M_PI * 2.0) / omega_max; 
        const double T_cruise = (circles_cruise * 2.0 * M_PI) / omega_max;       
        const double T_decel  = (circles_decel * 2.0 * M_PI * 2.0) / omega_max;    
        
        const double T_total = T_takeoff + T_accel + T_cruise + T_decel + T_land;
        const double T_wait_after_land = 3.0; 
        
        // ================= 任务结束统计与退出 =================
        if (t > T_total + T_wait_after_land) {
            
            // 🌟 打印盘旋阶段误差绝对值的平均值
            if (cruise_sample_count_ > 0) {
                double mae_x = sum_error_x_ / cruise_sample_count_;
                double mae_y = sum_error_y_ / cruise_sample_count_;
                double mae_z = sum_error_z_ / cruise_sample_count_;
                
                RCLCPP_INFO(this->get_logger(), "==================================================");
                RCLCPP_INFO(this->get_logger(), "📊 MAVROS 串级 - 【盘旋阶段】跟踪性能评估 (MAE):");
                RCLCPP_INFO(this->get_logger(), "   👉 采集样本数: %lu", cruise_sample_count_);
                RCLCPP_INFO(this->get_logger(), "   👉 X轴平均误差: %.4f m", mae_x);
                RCLCPP_INFO(this->get_logger(), "   👉 Y轴平均误差: %.4f m", mae_y);
                RCLCPP_INFO(this->get_logger(), "   👉 Z轴平均误差: %.4f m", mae_z);
                RCLCPP_INFO(this->get_logger(), "==================================================");
            }

            RCLCPP_INFO(this->get_logger(), "🏁 Mission Accomplished. Disarming and shutting down...");
            
            auto arm_cmd = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            arm_cmd->value = false;
            arming_client_->async_send_request(arm_cmd);
            
            rclcpp::shutdown(); 
            return; 
        }

        double px=0, py=0, pz=0;
        double vz=0, az=0;
        double theta=0, omega=0, alpha=0;

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
            double tau2 = tau * tau; double tau3 = tau2 * tau;
            double tau4 = tau3 * tau; double tau5 = tau4 * tau; double tau6 = tau5 * tau;
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
            double tau2 = tau * tau; double tau3 = tau2 * tau;
            double tau4 = tau3 * tau; double tau5 = tau4 * tau; double tau6 = tau5 * tau;
            double S = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
            double dS = (1.0 / T_decel) * (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4);

            pz = target_z; omega = omega_max * (1.0 - S); alpha = -omega_max * dS; 
            theta = (circles_accel + circles_cruise) * 2.0 * M_PI + (omega_max * T_decel * (tau - (2.5 * tau4 - 3.0 * tau5 + tau6))); 
        } 
        else if (t < T_total) {
            double tau = (t - T_takeoff - T_accel - T_cruise - T_decel) / T_land;
            double tau2 = tau * tau; double tau3 = tau2 * tau;
            double tau4 = tau3 * tau; double tau5 = tau4 * tau;
            double S = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
            double dS = (1.0 / T_land) * (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4);
            double ddS = (1.0 / std::pow(T_land, 2)) * (60.0 * tau - 180.0 * tau2 + 120.0 * tau3);

            pz = target_z * (1.0 - S); vz = -target_z * dS; az = -target_z * ddS; 
            theta = (circles_accel + circles_cruise + circles_decel) * 2.0 * M_PI; 
        }
        else {
            pz = 0; theta = (circles_accel + circles_cruise + circles_decel) * 2.0 * M_PI;
        }

        double c1 = std::cos(theta);
        double s1 = std::sin(theta);
        double c2 = std::cos(2.0 * theta);
        double s2 = std::sin(2.0 * theta);

        px = start_x_ + A * s1;
        py = start_y_ + B * s2;
        pz = start_z_ + pz;
        
        double vx = A * omega * c1;
        double vy = 2.0 * B * omega * c2;
        
        double ax = A * (alpha * c1 - std::pow(omega, 2) * s1);
        double ay = 2.0 * B * (alpha * c2 - 2.0 * std::pow(omega, 2) * s2);

        msg.position.x = px; msg.position.y = py; msg.position.z = pz;
        msg.velocity.x = vx; msg.velocity.y = vy; msg.velocity.z = vz;
        msg.acceleration_or_force.x = ax; msg.acceleration_or_force.y = ay; msg.acceleration_or_force.z = az;

        msg.yaw = start_yaw_; 

        setpoint_pub_->publish(msg);

        expected_path_.header.stamp = timestamp;
        path_pub_->publish(expected_path_);

        // 计算并发布实时位置误差
        geometry_msgs::msg::PointStamped error_msg;
        error_msg.header.stamp = timestamp;
        error_msg.header.frame_id = "map"; 
        
        if (has_pose_) {
            error_msg.point.x = current_pose_.pose.position.x - px;
            error_msg.point.y = current_pose_.pose.position.y - py;
            error_msg.point.z = current_pose_.pose.position.z - pz;

            // 🌟 仅在盘旋阶段统计平均绝对误差
            if (t >= T_takeoff + T_accel && t < T_takeoff + T_accel + T_cruise) {
                sum_error_x_ += std::abs(error_msg.point.x);
                sum_error_y_ += std::abs(error_msg.point.y);
                sum_error_z_ += std::abs(error_msg.point.z);
                cruise_sample_count_++;
            }
        }
        error_pub_->publish(error_msg);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), 
            *this->get_clock(), 
            500, 
            "🎯 MAVROS Tracking -> X: % .3f, Y: % .3f, Z: % .3f | Err(X:%.2f, Y:%.2f) | Mode Time: % .2f s", 
            px, py, pz, error_msg.point.x, error_msg.point.y, t
        );

        time_step_++;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MavrosAutoFigure8>());
    rclcpp::shutdown();
    return 0;
}