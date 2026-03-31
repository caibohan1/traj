#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <std_msgs/msg/float64_multi_array.hpp> // 完美适配 planner 的扩展通道
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
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        vehicle_command_pub_       = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        att_sp_pub_                = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);

        // --- Subscribers ---
        setpoint_sub_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
            "/geometric_controller/setpoint", 10,
            [this](const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) { 
                current_setpoint_ = *msg; 
                has_setpoint_ = true; 
                last_setpoint_time_ = this->get_clock()->now();
            });

        // 接收 Planner 发来的 Snap 和 YawAccel (本控制器输出姿态，暂不参与核心计算，但保持接口畅通)
        ext_setpoint_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/geometric_controller/traj_ext", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { 
                if(msg->data.size() >= 4) { current_ext_ = *msg; has_ext_ = true; } 
            });

        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(),
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) { 
                current_odom_ = *msg; 
                has_odom_ = true; 
                control_loop(); // 收到最新里程计后触发控制计算
            });
        
        RCLCPP_INFO(this->get_logger(), "Safe Geometric Controller (NED Fixed) Started.");
    }

private:
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr att_sp_pub_;
    
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ext_setpoint_sub_;

    px4_msgs::msg::VehicleOdometry current_odom_;
    px4_msgs::msg::TrajectorySetpoint current_setpoint_;
    std_msgs::msg::Float64MultiArray current_ext_;
    rclcpp::Time last_setpoint_time_;
    
    bool has_odom_ = false;
    bool has_setpoint_ = false;
    bool has_ext_ = false;
    uint64_t offboard_setpoint_counter_ = 0;
    
    double integral_error_z_ = 0.0;
    uint64_t last_odom_timestamp_ = 0; 

    void control_loop()
    {
        if (!has_odom_ || !has_setpoint_) return;

        publish_offboard_control_mode();

        // 超时保护：超过 0.5s 没收到轨迹
        if ((this->get_clock()->now() - last_setpoint_time_).seconds() > 0.5) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Setpoint timeout! Danger!");
        }

        // 强化解锁逻辑：在 50 到 100 周期之间，每 10 个周期发一次，确保飞控确实收到了解锁和切模式指令
        if (offboard_setpoint_counter_ >= 50 && offboard_setpoint_counter_ <= 100) { 
            if (offboard_setpoint_counter_ % 10 == 0) {
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // Offboard
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); // Arm
                if (offboard_setpoint_counter_ == 50) {
                    RCLCPP_INFO(this->get_logger(), "Sending Arm and Offboard commands...");
                }
            }
        }
        if (offboard_setpoint_counter_ < 150) offboard_setpoint_counter_++;

        compute_and_publish_control();
    }

    void compute_and_publish_control()
    {
        // ================= 物理参数 =================
        const double m = 1.535; // kg (适合 gz_x500)
        const double g = 9.81;  // m/s^2
        const double max_thrust = 24.0; // N
        const double max_tilt_angle = 45.0 * M_PI / 180.0; // 45 度安全倾角限制

        // 线性空气阻力系数矩阵
        Eigen::Matrix3d D;
        D << 0.25, 0.0,  0.0,
             0.0,  0.25, 0.0,
             0.0,  0.0,  0.0;

        // ================= 控制增益 =================
        Eigen::Matrix3d K_p = Eigen::Vector3d(3.0, 3.0, 8.0).asDiagonal();
        Eigen::Matrix3d K_v = Eigen::Vector3d(2.0, 2.0, 4.0).asDiagonal();
        double K_i_z = 2.0;

        // ================= 1. 提取当前状态 (NED) =================
        Eigen::Vector3d p(current_odom_.position[0], current_odom_.position[1], current_odom_.position[2]);
        Eigen::Vector3d v(current_odom_.velocity[0], current_odom_.velocity[1], current_odom_.velocity[2]);
        Eigen::Quaterniond q(current_odom_.q[0], current_odom_.q[1], current_odom_.q[2], current_odom_.q[3]);

        // ================= 2. 提取期望状态 (NED) =================
        Eigen::Vector3d p_d(current_setpoint_.position[0], current_setpoint_.position[1], current_setpoint_.position[2]);
        Eigen::Vector3d v_d(current_setpoint_.velocity[0], current_setpoint_.velocity[1], current_setpoint_.velocity[2]);
        Eigen::Vector3d a_d(current_setpoint_.acceleration[0], current_setpoint_.acceleration[1], current_setpoint_.acceleration[2]);
        double yaw_d = current_setpoint_.yaw;

        // ================= 3. 位置环：计算期望加速度与推力 =================
        Eigen::Vector3d e_p = p - p_d;
        Eigen::Vector3d e_v = v - v_d;
        
        double dt = 0.01; 
        if (last_odom_timestamp_ != 0 && current_odom_.timestamp > last_odom_timestamp_) {
            dt = static_cast<double>(current_odom_.timestamp - last_odom_timestamp_) / 1e6; 
        }
        last_odom_timestamp_ = current_odom_.timestamp;
        if (dt > 0.05) dt = 0.01; 

        // 高度积分防饱和
        integral_error_z_ += e_p.z() * dt;
        integral_error_z_ = std::clamp(integral_error_z_, -3.0, 3.0); 

        // 期望加速度 (包含 PD 反馈、前馈、Z轴积分)
        Eigen::Vector3d a_cmd = a_d - K_p * e_p - K_v * e_v;
        a_cmd.z() -= K_i_z * integral_error_z_;

        // 基础期望推力矢量 (无阻力)：T_vec = m * (g * e_3 - a_cmd)
        Eigen::Vector3d e_3(0.0, 0.0, 1.0);
        Eigen::Vector3d T_vec_pre = m * (g * e_3 - a_cmd);

        // 利用基础推力计算初始姿态，用于阻力坐标系转换
        Eigen::Vector3d z_b_pre = T_vec_pre.normalized();
        Eigen::Vector3d x_c(std::cos(yaw_d), std::sin(yaw_d), 0.0);
        Eigen::Vector3d y_b_pre = z_b_pre.cross(x_c).normalized();
        Eigen::Vector3d x_b_pre = y_b_pre.cross(z_b_pre);
        Eigen::Matrix3d R_pre;
        R_pre << x_b_pre, y_b_pre, z_b_pre;

        // 引入空气阻力前馈补偿：a_drag = R * D * R^T * v_d
        Eigen::Vector3d a_drag = R_pre * D * R_pre.transpose() * v_d;
        
        // 最终期望推力矢量
        Eigen::Vector3d T_vec = m * (g * e_3 - a_cmd - a_drag);

        // ================= 4. 安全硬限幅处理 (Output Clamping) =================
        // 4.1 防止推力反向 (NED坐标系中，机体Z轴应当保持朝下，所以T_vec.z()必须为正数)
        // 修复了之前的反向钳制 Bug！
        if (T_vec.z() < 0.1) T_vec.z() = 0.1; 
        
        // 4.2 最大倾角圆锥钳制
        double current_tilt_cos = std::abs(T_vec.z()) / T_vec.norm();
        if (current_tilt_cos < std::cos(max_tilt_angle)) {
            double max_xy_mag = std::abs(T_vec.z()) * std::tan(max_tilt_angle);
            double xy_mag = std::sqrt(T_vec.x() * T_vec.x() + T_vec.y() * T_vec.y());
            T_vec.x() *= max_xy_mag / xy_mag;
            T_vec.y() *= max_xy_mag / xy_mag;
        }

        double T_norm = T_vec.norm();

        // ================= 5. 几何映射：最终期望姿态 R_d (FRD) =================
        Eigen::Vector3d z_b_d = T_vec.normalized(); 
        Eigen::Vector3d y_b_d = z_b_d.cross(x_c).normalized();
        Eigen::Vector3d x_b_d = y_b_d.cross(z_b_d);
        
        Eigen::Matrix3d R_d;
        R_d << x_b_d, y_b_d, z_b_d; 
        Eigen::Quaterniond q_d(R_d);

        // ================= 6. 归一化并发布 =================
        publish_attitude_setpoint(q_d, T_norm, max_thrust);
    }

    void publish_attitude_setpoint(const Eigen::Quaterniond& q_d, double T_norm, double max_thrust)
    {
        px4_msgs::msg::VehicleAttitudeSetpoint msg{};
        msg.timestamp = current_odom_.timestamp;
        
        msg.q_d[0] = q_d.w();
        msg.q_d[1] = q_d.x();
        msg.q_d[2] = q_d.y();
        msg.q_d[3] = q_d.z();
        
        double normalized_thrust = std::clamp(T_norm / max_thrust, 0.05, 0.95);

        // FRD坐标系下，推力沿机体 Z 轴负方向喷射（向上抬升）
        msg.thrust_body[0] = 0.0;
        msg.thrust_body[1] = 0.0;
        msg.thrust_body[2] = -normalized_thrust; 

        att_sp_pub_->publish(msg);
    }

    void publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = current_odom_.timestamp; 
        msg.position = false;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = true;  
        msg.body_rate = false;
        msg.thrust_and_torque = false; 
        offboard_control_mode_pub_->publish(msg);
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = current_odom_.timestamp; 
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 191; 
        msg.from_external = true;
        vehicle_command_pub_->publish(msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GeometricController>());
    rclcpp::shutdown();
    return 0;
}