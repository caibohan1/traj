#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;

class MavrosGeometricController : public rclcpp::Node
{
public:
    MavrosGeometricController() : Node("mavros_geometric_controller")
    {
        // --- 发布者 ---
        // 发布角速度和推力给 MAVROS
        att_sp_pub_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);

        // --- 服务客户端 ---
        arming_client_   = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

        // --- 订阅者 ---
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            [this](const mavros_msgs::msg::State::SharedPtr msg) { current_state_ = *msg; });

        // 订阅期望轨迹 (来自规划器)
        setpoint_sub_ = this->create_subscription<mavros_msgs::msg::PositionTarget>(
            "/geometric_controller/setpoint", 10,
            [this](const mavros_msgs::msg::PositionTarget::SharedPtr msg) { 
                current_setpoint_ = *msg; 
                has_setpoint_ = true; 
                last_setpoint_time_ = this->get_clock()->now(); 
            });

        // 订阅 MAVROS 里程计 (ENU)
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/mavros/local_position/odom", rclcpp::QoS(10).best_effort(),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) { 
                current_odom_ = *msg; 
                has_odom_ = true; 
            });
        
        // 250Hz 控制循环
        timer_ = this->create_wall_timer(4ms, std::bind(&MavrosGeometricController::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Geometric Controller (MAVROS/ENU) with Failsafe Started.");
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
    rclcpp::TimerBase::SharedPtr timer_;
    
    bool has_odom_ = false;
    bool has_setpoint_ = false;
    uint64_t offboard_counter_ = 0;
    rclcpp::Time last_setpoint_time_; 
    
    double integral_error_z_ = 0.0;

    Eigen::Vector3d vee_map(const Eigen::Matrix3d& S) const {
        return Eigen::Vector3d(S(2, 1), S(0, 2), S(1, 0));
    }

    void control_loop()
    {
        if (!has_odom_) return;

        // --- 安全看门狗 (Failsafe) ---
        if (has_setpoint_) {
            double dt_timeout = (this->get_clock()->now() - last_setpoint_time_).seconds();
            if (dt_timeout > 0.5) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Setpoint Timeout!");
                
                // 判断是否在地面 (ENU 坐标系，Z < 0.5m)
                bool is_grounded = (current_odom_.pose.pose.position.z < 0.5) &&
                                   (std::abs(current_odom_.twist.twist.linear.x) < 0.2) &&
                                   (std::abs(current_odom_.twist.twist.linear.z) < 0.2);

                if (is_grounded && current_state_.armed) {
                    auto disarm_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                    disarm_req->value = false;
                    arming_client_->async_send_request(disarm_req);
                } else if (!is_grounded) {
                    auto mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                    mode_req->custom_mode = "AUTO.LOITER";
                    set_mode_client_->async_send_request(mode_req);
                }
                return;
            }
        } else return;

        // --- 自动切换模式与解锁逻辑 ---
        if (offboard_counter_ == 50) { // 提前发送一些指令流再切 Offboard
            auto mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            mode_req->custom_mode = "OFFBOARD";
            set_mode_client_->async_send_request(mode_req);
            
            auto arm_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            arm_req->value = true;
            arming_client_->async_send_request(arm_req);
            RCLCPP_INFO(this->get_logger(), "Requesting OFFBOARD and ARM...");
        }
        if (offboard_counter_ < 100) offboard_counter_++;

        compute_and_publish_control();
    }

    void compute_and_publish_control()
    {
        const double m = 1.535; 
        const double g = 9.81;  
        const double max_thrust = 28.467; 

        // 阻力系数
        Eigen::Matrix3d D;
        D << 0.15, 0.0,  0.0,
             0.0,  0.15, 0.0,
             0.0,  0.0,  0.0;

        // 控制增益
        Eigen::Matrix3d K_p = Eigen::Vector3d(3.5, 3.5, 8.0).asDiagonal();
        Eigen::Matrix3d K_v = Eigen::Vector3d(2.0, 2.0, 4.0).asDiagonal();
        Eigen::Matrix3d K_R = Eigen::Vector3d(6.0, 6.0, 3.0).asDiagonal();
        double K_i_z = 2.0;

        // --- 1. 提取当前状态 ---
        Eigen::Vector3d p(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z);
        Eigen::Quaterniond q(current_odom_.pose.pose.orientation.w, current_odom_.pose.pose.orientation.x, 
                             current_odom_.pose.pose.orientation.y, current_odom_.pose.pose.orientation.z);
        Eigen::Matrix3d R = q.toRotationMatrix();
        
        // MAVROS twist linear 是在机体系 FLU，需要旋转到世界系 ENU
        Eigen::Vector3d v_body(current_odom_.twist.twist.linear.x, current_odom_.twist.twist.linear.y, current_odom_.twist.twist.linear.z);
        Eigen::Vector3d v = R * v_body; 

        // --- 2. 提取期望状态 ---
        Eigen::Vector3d p_d(current_setpoint_.position.x, current_setpoint_.position.y, current_setpoint_.position.z);
        Eigen::Vector3d v_d(current_setpoint_.velocity.x, current_setpoint_.velocity.y, current_setpoint_.velocity.z);
        Eigen::Vector3d a_d(current_setpoint_.acceleration_or_force.x, current_setpoint_.acceleration_or_force.y, current_setpoint_.acceleration_or_force.z);
        
        // MAVROS PositionTarget 消息中没有 Jerk 字段，默认设为 0（除非自定义消息）
        Eigen::Vector3d j_d(0, 0, 0); 
        double yaw_d = current_setpoint_.yaw;
        double yawspeed_d = current_setpoint_.yaw_rate;

        // --- 3. 位置环：计算期望力矢量 ---
        Eigen::Vector3d e_p = p - p_d;
        Eigen::Vector3d e_v = v - v_d;
        
        integral_error_z_ += e_p.z() * 0.004; 
        integral_error_z_ = std::clamp(integral_error_z_, -6.0, 6.0);
    
        // ENU 下，抵消重力需要向上的加速度 +g
        Eigen::Vector3d F_des_pre = -K_p * e_p - K_v * e_v + m * a_d + Eigen::Vector3d(0, 0, m * g);
        
        // 计算期望旋转矩阵以便进行阻力补偿
        Eigen::Vector3d z_b_d_pre = F_des_pre.normalized();
        Eigen::Vector3d x_c(std::cos(yaw_d), std::sin(yaw_d), 0.0);
        Eigen::Vector3d y_b_d_pre = z_b_d_pre.cross(x_c).normalized();
        Eigen::Vector3d x_b_d_pre = y_b_d_pre.cross(z_b_d_pre);
        Eigen::Matrix3d R_d_pre; R_d_pre << x_b_d_pre, y_b_d_pre, z_b_d_pre;

        Eigen::Vector3d F_drag = R_d_pre * D * R_d_pre.transpose() * v_d;
        
        // 最终期望推力矢量
        Eigen::Vector3d T_vec = F_des_pre + F_drag;
        T_vec.z() -= K_i_z * integral_error_z_;

        double T_norm = T_vec.norm();

        // --- 4. 几何映射：计算 R_des ---
        Eigen::Vector3d z_b_d = T_vec.normalized(); 
        Eigen::Vector3d y_b_d = z_b_d.cross(x_c).normalized();
        Eigen::Vector3d x_b_d = y_b_d.cross(z_b_d);
        Eigen::Matrix3d R_d; R_d << x_b_d, y_b_d, z_b_d; 

        // 姿态误差 e_R
        Eigen::Matrix3d err_matrix = 0.5 * (R_d.transpose() * R - R.transpose() * R_d);
        Eigen::Vector3d e_R = vee_map(err_matrix);

        // --- 5. 微分平坦性映射 (生成期望角速度) ---
        // 此部分推导基于 ENU
        double c = T_norm / m;
        double d_x = D(0,0), d_y = D(1,1), d_z = D(2,2);
        
        double v_x = x_b_d.dot(v_d), v_y = y_b_d.dot(v_d), v_z = z_b_d.dot(v_d);
        double a_x = x_b_d.dot(a_d), a_y = y_b_d.dot(a_d);
        double j_x = x_b_d.dot(j_d), j_y = y_b_d.dot(j_d);

        Eigen::Vector3d y_c(-std::sin(yaw_d), std::cos(yaw_d), 0.0);
        Eigen::Vector3d y_c_cross_z_b_d = y_c.cross(z_b_d);

        Eigen::Matrix3d M_omega;
        M_omega.setZero();
        M_omega(0, 1) = c - (d_z - d_x) * v_z;
        M_omega(0, 2) = -(d_x - d_y) * v_y;
        M_omega(1, 0) = -(c + (d_y - d_z) * v_z);
        M_omega(1, 2) = (d_x - d_y) * v_x;
        M_omega(2, 1) = -y_c.dot(z_b_d);
        M_omega(2, 2) = y_c_cross_z_b_d.norm();

        Eigen::Vector3d b_omega;
        b_omega(0) = j_x + d_x * a_x;
        b_omega(1) = -j_y - d_y * a_y;
        b_omega(2) = yawspeed_d * x_c.dot(x_b_d);

        Eigen::Vector3d omega_d = M_omega.inverse() * b_omega;

        // --- 6. 姿态环指令 ---
        Eigen::Vector3d omega_ff = R.transpose() * R_d * omega_d;
        Eigen::Vector3d omega_cmd = omega_ff - K_R * e_R;

        // --- 7. 推力投影与发布 ---
        // 在 FLU 系下，推力沿机体 Z 轴，故点乘 R 矩阵的第三列
        double f = T_vec.dot(R.col(2)); 
        
        mavros_msgs::msg::AttitudeTarget out_msg;
        out_msg.header.stamp = this->get_clock()->now();
        out_msg.header.frame_id = "base_link";
        // 忽略姿态，直接控制角速度和推力
        out_msg.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ATTITUDE;
        out_msg.body_rate.x = omega_cmd.x();
        out_msg.body_rate.y = omega_cmd.y();
        out_msg.body_rate.z = omega_cmd.z();
        out_msg.thrust = std::clamp(f / max_thrust, 0.05, 0.95);

        att_sp_pub_->publish(out_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MavrosGeometricController>());
    rclcpp::shutdown();
    return 0;
}