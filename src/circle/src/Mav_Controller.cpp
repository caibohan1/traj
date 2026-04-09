#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

#include <Eigen/Dense>
#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;

class GeometricController : public rclcpp::Node
{
public:
    GeometricController() : Node("geometric_controller")
    {
        // --- Publishers ---
        att_sp_pub_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);

        // --- Service Clients ---
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

        // --- Subscribers ---
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            [this](const mavros_msgs::msg::State::SharedPtr msg) { current_state_ = *msg; });

        setpoint_sub_ = this->create_subscription<mavros_msgs::msg::PositionTarget>(
            "/geometric_controller/setpoint", 10,
            [this](const mavros_msgs::msg::PositionTarget::SharedPtr msg) { 
                current_setpoint_ = *msg; 
                has_setpoint_ = true; 
                last_setpoint_time_ = this->get_clock()->now();
            });

        // 订阅 MAVROS 的本地里程计 (ENU)
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/mavros/local_position/odom", rclcpp::QoS(10).best_effort(),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) { 
                current_odom_ = *msg; 
                has_odom_ = true; 
                control_loop(); 
            });
        
        RCLCPP_INFO(this->get_logger(), "Geometric Controller (MAVROS/ENU) Started.");
    }

private:
    rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr att_sp_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;

    nav_msgs::msg::Odometry current_odom_;
    mavros_msgs::msg::PositionTarget current_setpoint_;
    mavros_msgs::msg::State current_state_;
    
    rclcpp::Time last_setpoint_time_;
    bool has_odom_ = false;
    bool has_setpoint_ = false;

    uint64_t offboard_setpoint_counter_ = 0;
    
    double integral_error_z_ = 0.0;
    rclcpp::Time last_odom_timestamp_; 

    void control_loop()
    {
        if (!has_odom_ || !has_setpoint_) return;

        // 超时保护
        if ((this->get_clock()->now() - last_setpoint_time_).seconds() > 0.5) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Setpoint timeout!");
            // ENU 坐标系下，Z < 0.3 代表在地面
            bool is_grounded = (current_odom_.pose.pose.position.z < 0.3) &&
                               (std::abs(current_odom_.twist.twist.linear.x) < 0.2) &&
                               (std::abs(current_odom_.twist.twist.linear.y) < 0.2) &&
                               (std::abs(current_odom_.twist.twist.linear.z) < 0.2);

            if (is_grounded && current_state_.armed) {
                RCLCPP_INFO(this->get_logger(), "Planner offline and drone is grounded. Disarming...");
                auto arm_cmd = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                arm_cmd->value = false;
                arming_client_->async_send_request(arm_cmd);
                rclcpp::shutdown(); 
                return;
            }
        }

        // 必须先持续发送期望值，再切入 Offboard
        compute_and_publish_control();

        // Offboard 与解锁逻辑 (使用 MAVROS 服务)
        if (offboard_setpoint_counter_ < 100) {
            offboard_setpoint_counter_++;
        } else if (offboard_setpoint_counter_ == 100) {
            if (current_state_.mode != "OFFBOARD") {
                auto set_mode_cmd = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                set_mode_cmd->custom_mode = "OFFBOARD";
                set_mode_client_->async_send_request(set_mode_cmd);
                RCLCPP_INFO(this->get_logger(), "Requesting OFFBOARD...");
            }
            if (!current_state_.armed) {
                auto arm_cmd = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                arm_cmd->value = true;
                arming_client_->async_send_request(arm_cmd);
                RCLCPP_INFO(this->get_logger(), "Requesting ARM...");
            }
            offboard_setpoint_counter_++;
        }
    }

void compute_and_publish_control()
    {
        const double m = 1.535; // kg 
        const double g = 9.81;  // m/s^2
        const double max_thrust = 25.1; // N
        // 恢复到你原有的合理限幅 45 度
        const double max_tilt_angle = 45.0 * M_PI / 180.0; 

        // 暂时保留阻力前馈，如果环境风不大，这段也可以置 0
        Eigen::Matrix3d D;
        D << 0.25, 0.0,  0.0,
             0.0,  0.25, 0.0,
             0.0,  0.0,  0.0;

        // 使用你原来的增益，既然起飞稳定说明增益基础是对的
        Eigen::Matrix3d K_p = Eigen::Vector3d(3.0, 3.0, 8.0).asDiagonal();
        Eigen::Matrix3d K_v = Eigen::Vector3d(2.0, 2.0, 4.0).asDiagonal();
        double K_i_z = 2.0;

        // ================= 1. 提取当前状态 =================
        // 位置在 ENU 世界坐标系
        Eigen::Vector3d p(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z);
        // 姿态是从 ENU 到 FLU 的旋转
        Eigen::Quaterniond q(current_odom_.pose.pose.orientation.w, current_odom_.pose.pose.orientation.x, 
                             current_odom_.pose.pose.orientation.y, current_odom_.pose.pose.orientation.z);

        // 【核心修复】：提取机体坐标系(FLU)下的线速度
        Eigen::Vector3d v_body(current_odom_.twist.twist.linear.x, current_odom_.twist.twist.linear.y, current_odom_.twist.twist.linear.z);
        // 使用四元数将其旋转到世界坐标系(ENU)
        Eigen::Vector3d v = q * v_body; 

        // ================= 2. 提取期望状态 (ENU) =================
        Eigen::Vector3d p_d(current_setpoint_.position.x, current_setpoint_.position.y, current_setpoint_.position.z);
        Eigen::Vector3d v_d(current_setpoint_.velocity.x, current_setpoint_.velocity.y, current_setpoint_.velocity.z);
        Eigen::Vector3d a_d(current_setpoint_.acceleration_or_force.x, current_setpoint_.acceleration_or_force.y, current_setpoint_.acceleration_or_force.z);
        double yaw_d = current_setpoint_.yaw;

        // ================= 3. 位置环计算 =================
        Eigen::Vector3d e_p = p - p_d;
        Eigen::Vector3d e_v = v - v_d; // 现在两边都是 ENU 坐标系了，相减成立！
        
        // 修复时间戳逻辑：使用消息自带的精确时间
        rclcpp::Time current_time = current_odom_.header.stamp;
        double dt = 0.01; 
        if (last_odom_timestamp_.nanoseconds() != 0) {
            dt = (current_time - last_odom_timestamp_).seconds(); 
        }
        last_odom_timestamp_ = current_time;
        if (dt > 0.05 || dt <= 0.0) dt = 0.01; 

        // Z 轴积分防饱和
        integral_error_z_ += e_p.z() * dt;
        integral_error_z_ = std::clamp(integral_error_z_, -3.0, 3.0); 

        // 期望加速度
        Eigen::Vector3d a_cmd = a_d - K_p * e_p - K_v * e_v;
        a_cmd.z() -= K_i_z * integral_error_z_;

        // 在 ENU 中抵消向下的重力，需要加一个向上的加速度分量
        Eigen::Vector3d e_3(0.0, 0.0, 1.0);
        Eigen::Vector3d T_vec_pre = m * (a_cmd + g * e_3);

        // 计算带阻力前馈的推力
        Eigen::Vector3d z_b_pre = T_vec_pre.normalized();
        Eigen::Vector3d x_c(std::cos(yaw_d), std::sin(yaw_d), 0.0);
        Eigen::Vector3d y_b_pre = z_b_pre.cross(x_c).normalized();
        Eigen::Vector3d x_b_pre = y_b_pre.cross(z_b_pre);
        Eigen::Matrix3d R_pre;
        R_pre << x_b_pre, y_b_pre, z_b_pre;

        Eigen::Vector3d a_drag = R_pre * D * R_pre.transpose() * v_d;
        Eigen::Vector3d T_vec = m * (a_cmd + g * e_3) + a_drag;

        // ================= 4. 安全限幅 =================
        if (T_vec.z() < 0.1) T_vec.z() = 0.1; 
        
        double current_tilt_cos = std::abs(T_vec.z()) / T_vec.norm();
        if (current_tilt_cos < std::cos(max_tilt_angle)) {
            double max_xy_mag = std::abs(T_vec.z()) * std::tan(max_tilt_angle);
            double xy_mag = std::sqrt(T_vec.x() * T_vec.x() + T_vec.y() * T_vec.y());
            T_vec.x() *= max_xy_mag / xy_mag;
            T_vec.y() *= max_xy_mag / xy_mag;
        }

        double T_norm = T_vec.norm();

        // ================= 5. 几何映射 =================
        Eigen::Vector3d z_b_d = T_vec.normalized(); 
        Eigen::Vector3d y_b_d = z_b_d.cross(x_c).normalized();
        Eigen::Vector3d x_b_d = y_b_d.cross(z_b_d);
        
        Eigen::Matrix3d R_d;
        R_d << x_b_d, y_b_d, z_b_d; 
        Eigen::Quaterniond q_d(R_d);

        publish_attitude_setpoint(q_d, T_norm, max_thrust);
    }
    
    void publish_attitude_setpoint(const Eigen::Quaterniond& q_d, double T_norm, double max_thrust)
    {
        mavros_msgs::msg::AttitudeTarget msg{};
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "base_link";
        
        msg.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
                        mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
                        mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;

        msg.orientation.w = q_d.w();
        msg.orientation.x = q_d.x();
        msg.orientation.y = q_d.y();
        msg.orientation.z = q_d.z();
        
        // MAVROS 要求推力是一个标量 [0, 1]
        msg.thrust = std::clamp(T_norm / max_thrust, 0.05, 0.95);

        att_sp_pub_->publish(msg);

        // 🌟 新增：打印期望姿态四元数和发送给 MAVROS 的标量推力（每 500 毫秒打印一次）
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            500,
            "📐 MAVROS Att Setpoint -> q: [%.2f, %.2f, %.2f, %.2f] | Raw Thrust(N): %.2f | Scaled Thrust: %.2f",
            q_d.w(), q_d.x(), q_d.y(), q_d.z(), T_norm, msg.thrust
        );
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GeometricController>());
    rclcpp::shutdown();
    return 0;
}