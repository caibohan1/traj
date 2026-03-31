#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <std_msgs/msg/float64_multi_array.hpp> 
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
        thrust_pub_                = this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>("/fmu/in/vehicle_thrust_setpoint", 10);
        torque_pub_                = this->create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>("/fmu/in/vehicle_torque_setpoint", 10);

        // --- Subscribers ---
        setpoint_sub_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
            "/geometric_controller/setpoint", 10,
            [this](const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) { current_setpoint_ = *msg; has_setpoint_ = true; });

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
                control_loop(); // 收到最新状态后，立即触发控制计算
            });
        
        RCLCPP_INFO(this->get_logger(), "Geometric Controller with NED-ENU Wrapper Started.");
    }

private:
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr torque_pub_;
    
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ext_setpoint_sub_; 

    px4_msgs::msg::VehicleOdometry current_odom_;
    px4_msgs::msg::TrajectorySetpoint current_setpoint_;
    std_msgs::msg::Float64MultiArray current_ext_; 
    
    bool has_odom_ = false;
    bool has_setpoint_ = false;
    bool has_ext_ = false; 
    uint64_t offboard_setpoint_counter_ = 0;
    
    double integral_error_z_ = 0.0;
    uint64_t last_odom_timestamp_ = 0; 

    // 反对称矩阵的 Vee 映射 
    Eigen::Vector3d vee_map(const Eigen::Matrix3d& S) const {
        return Eigen::Vector3d(S(2, 1), S(0, 2), S(1, 0));
    }

    // 向量到反对称矩阵映射 (Hat Map)
    Eigen::Matrix3d hat_map(const Eigen::Vector3d& v) const {
        Eigen::Matrix3d S;
        S <<  0.0,  -v(2),  v(1),
              v(2),   0.0, -v(0),
             -v(1),  v(0),   0.0;
        return S;
    }

    void control_loop()
    {
        if (!has_odom_ || !has_setpoint_ || !has_ext_) return;

        publish_offboard_control_mode();

        if (offboard_setpoint_counter_ == 50) { 
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // Offboard
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); // Arm
            RCLCPP_INFO(this->get_logger(), "Arming and setting Offboard mode!");
        }
        if (offboard_setpoint_counter_ < 100) offboard_setpoint_counter_++;

        compute_and_publish_control();
    }

    void compute_and_publish_control()
    {
        // ================= 物理参数 =================
        const double m = 1.535; // kg (根据实际 x500 调整)
        const double g = 9.81;  // m/s^2
        const double max_thrust = 24.0; // N
        const Eigen::Vector3d max_torque(0.5, 0.5, 0.5); // Nm
        
        Eigen::Matrix3d J;
        J << 0.016, 0.0,   0.0,
             0.0,   0.016, 0.0,
             0.0,   0.0,   0.021;

        // 【阻力系数矩阵】测试阶段建议先给较小的值，避免直接发散
        Eigen::Matrix3d D;
        D << 0.25, 0.0,  0.0,
             0.0,  0.25, 0.0,
             0.0,  0.0,  0.0;

        // ================= 控制增益 =================
        Eigen::Matrix3d K_p = Eigen::Vector3d(2.5, 2.5, 8.0).asDiagonal();
        Eigen::Matrix3d K_v = Eigen::Vector3d(2.0, 2.0, 4.0).asDiagonal();
        double K_i_z = 2.0;

        Eigen::Matrix3d K_R = Eigen::Vector3d(0.8, 0.8, 5.5).asDiagonal();
        Eigen::Matrix3d K_w = Eigen::Vector3d(0.08, 0.08, 0.55).asDiagonal();

        // ================= 1. 提取当前状态 (NED/FRD) =================
        Eigen::Vector3d p(current_odom_.position[0], current_odom_.position[1], current_odom_.position[2]);
        Eigen::Vector3d v(current_odom_.velocity[0], current_odom_.velocity[1], current_odom_.velocity[2]);
        Eigen::Quaterniond q(current_odom_.q[0], current_odom_.q[1], current_odom_.q[2], current_odom_.q[3]);
        Eigen::Matrix3d R = q.toRotationMatrix();
        Eigen::Vector3d omega(current_odom_.angular_velocity[0], current_odom_.angular_velocity[1], current_odom_.angular_velocity[2]);

        // ================= 2. 提取期望状态 (NED) =================
        Eigen::Vector3d p_d(current_setpoint_.position[0], current_setpoint_.position[1], current_setpoint_.position[2]);
        Eigen::Vector3d v_d(current_setpoint_.velocity[0], current_setpoint_.velocity[1], current_setpoint_.velocity[2]);
        Eigen::Vector3d a_d(current_setpoint_.acceleration[0], current_setpoint_.acceleration[1], current_setpoint_.acceleration[2]);
        Eigen::Vector3d j_d(current_setpoint_.jerk[0], current_setpoint_.jerk[1], current_setpoint_.jerk[2]); 
        Eigen::Vector3d snap_d(current_ext_.data[0], current_ext_.data[1], current_ext_.data[2]);
        
        double yaw_d = current_setpoint_.yaw;
        double yawspeed_d = current_setpoint_.yawspeed;
        double yawaccel_d = current_ext_.data[3];

        // ================= 3. 位置环：计算期望推力 (NED) =================
        Eigen::Vector3d e_p = p - p_d;
        Eigen::Vector3d e_v = v - v_d;
        
        double dt = 0.01; 
        if (last_odom_timestamp_ != 0 && current_odom_.timestamp > last_odom_timestamp_) {
            dt = static_cast<double>(current_odom_.timestamp - last_odom_timestamp_) / 1e6; 
        }
        last_odom_timestamp_ = current_odom_.timestamp;
        
        if (dt > 0.05) dt = 0.01; 

        integral_error_z_ += e_p.z() * dt;
        integral_error_z_ = std::clamp(integral_error_z_, -6.0, 6.0);
    
        // 计算期望姿态的初步猜测（用于更精确地补偿阻力）
        Eigen::Vector3d F_des_pre = -K_p * e_p - K_v * e_v + m * a_d - Eigen::Vector3d(0, 0, m * g);
        Eigen::Vector3d z_b_d_pre = (-F_des_pre).normalized();
        Eigen::Vector3d x_c(std::cos(yaw_d), std::sin(yaw_d), 0.0);
        Eigen::Vector3d y_b_d_pre = z_b_d_pre.cross(x_c).normalized();
        Eigen::Vector3d x_b_d_pre = y_b_d_pre.cross(z_b_d_pre);
        Eigen::Matrix3d R_d_pre;
        R_d_pre << x_b_d_pre, y_b_d_pre, z_b_d_pre;

        // 引入阻力补偿：F_drag = + m * R_d * D * R_d^T * v_d (严格对应论文中的 -a_rd)
        Eigen::Vector3d F_drag = m * R_d_pre * D * R_d_pre.transpose() * v_d;
        
        Eigen::Vector3d F_des = -K_p * e_p - K_v * e_v + m * a_d - Eigen::Vector3d(0, 0, m * g) + F_drag;
        F_des.z() -= K_i_z * integral_error_z_;

        Eigen::Vector3d T_vec = -F_des; 
        double T_norm = T_vec.norm();

        // ================= 4. 几何映射：最终期望姿态 R_d (FRD) =================
        Eigen::Vector3d z_b_d = T_vec.normalized(); 
        Eigen::Vector3d y_c(-std::sin(yaw_d), std::cos(yaw_d), 0.0);
        Eigen::Vector3d y_b_d = z_b_d.cross(x_c).normalized();
        Eigen::Vector3d x_b_d = y_b_d.cross(z_b_d);
        
        Eigen::Matrix3d R_d;
        R_d << x_b_d, y_b_d, z_b_d; 

        Eigen::Matrix3d err_matrix = 0.5 * (R_d.transpose() * R - R.transpose() * R_d);
        Eigen::Vector3d e_R = vee_map(err_matrix);

        // ================= 5. 微分平坦性：NED - ENU - NED 包装器 =================
        
        // [5.1] 输入转换：NED (北东下) -> ENU (东北天)
        Eigen::Vector3d v_d_enu(v_d.x(), -v_d.y(), -v_d.z());
        Eigen::Vector3d a_d_enu(a_d.x(), -a_d.y(), -a_d.z());
        Eigen::Vector3d j_d_enu(j_d.x(), -j_d.y(), -j_d.z());
        Eigen::Vector3d snap_d_enu(snap_d.x(), -snap_d.y(), -snap_d.z());
        double yaw_d_enu = -yaw_d;
        double yawspeed_d_enu = -yawspeed_d;
        double yawaccel_d_enu = -yawaccel_d;

        // 机体坐标系轴转换：FRD -> FLU
        Eigen::Vector3d z_b_d_enu(z_b_d.x(), -z_b_d.y(), -z_b_d.z());
        Eigen::Vector3d x_c_enu(std::cos(yaw_d_enu), std::sin(yaw_d_enu), 0.0);
        Eigen::Vector3d y_c_enu(-std::sin(yaw_d_enu), std::cos(yaw_d_enu), 0.0);
        
        Eigen::Vector3d y_b_d_enu = z_b_d_enu.cross(x_c_enu).normalized();
        Eigen::Vector3d x_b_d_enu = y_b_d_enu.cross(z_b_d_enu);

        Eigen::Matrix3d R_d_enu;
        R_d_enu << x_b_d_enu, y_b_d_enu, z_b_d_enu;

        // [5.2] 在 ENU 坐标系下执行论文原汁原味的平坦性推导
        double d_x = D(0,0);
        double d_y = D(1,1);
        double d_z = D(2,2);
        double c = T_norm / m; 
        
        double v_x = x_b_d_enu.dot(v_d_enu); double v_y = y_b_d_enu.dot(v_d_enu); double v_z = z_b_d_enu.dot(v_d_enu);
        double a_x = x_b_d_enu.dot(a_d_enu); double a_y = y_b_d_enu.dot(a_d_enu);
        double j_x = x_b_d_enu.dot(j_d_enu); double j_y = y_b_d_enu.dot(j_d_enu);

        Eigen::Vector3d y_c_cross_z_b_d = y_c_enu.cross(z_b_d_enu);

        Eigen::Matrix3d M_omega;
        M_omega.setZero();
        M_omega(0, 1) = c - (d_z - d_x) * v_z;
        M_omega(0, 2) = -(d_x - d_y) * v_y;
        M_omega(1, 0) = -(c + (d_y - d_z) * v_z);
        M_omega(1, 2) = (d_x - d_y) * v_x;
        M_omega(2, 1) = -y_c_enu.dot(z_b_d_enu);
        M_omega(2, 2) = y_c_cross_z_b_d.norm();

        Eigen::Matrix3d M_inv = M_omega.inverse();

        Eigen::Vector3d b_omega;
        b_omega(0) = j_x + d_x * a_x;
        b_omega(1) = -j_y - d_y * a_y;
        b_omega(2) = yawspeed_d_enu * x_c_enu.dot(x_b_d_enu);
        Eigen::Vector3d omega_d_enu = M_inv * b_omega;

        Eigen::Matrix3d o_hat = hat_map(omega_d_enu);
        Eigen::Matrix3d o_hat_sq = o_hat * o_hat;
        Eigen::Matrix3d o_hat_T = o_hat.transpose();

        Eigen::Matrix3d term1 = o_hat_sq * D + D * o_hat_sq + 2.0 * o_hat * D * o_hat_T;
        Eigen::Matrix3d term2 = o_hat * D + D * o_hat_T;
        Eigen::Vector3d xi = R_d_enu * term1 * R_d_enu.transpose() * v_d_enu 
                           + 2.0 * R_d_enu * term2 * R_d_enu.transpose() * a_d_enu 
                           + R_d_enu * D * R_d_enu.transpose() * j_d_enu;

        double c_dot = z_b_d_enu.dot(j_d_enu) + omega_d_enu(0) * (d_y - d_z) * y_b_d_enu.dot(v_d_enu) 
                     + omega_d_enu(1) * (d_z - d_x) * x_b_d_enu.dot(v_d_enu) + d_z * z_b_d_enu.dot(a_d_enu);

        Eigen::Vector3d b_dot_omega;
        b_dot_omega(0) = x_b_d_enu.dot(snap_d_enu) - 2.0 * c_dot * omega_d_enu(1) - c * omega_d_enu(0) * omega_d_enu(2) + x_b_d_enu.dot(xi);
        b_dot_omega(1) = -y_b_d_enu.dot(snap_d_enu) - 2.0 * c_dot * omega_d_enu(0) + c * omega_d_enu(1) * omega_d_enu(2) - y_b_d_enu.dot(xi);
        b_dot_omega(2) = yawaccel_d_enu * x_c_enu.dot(x_b_d_enu) + 2.0 * yawspeed_d_enu * omega_d_enu(2) * x_c_enu.dot(y_b_d_enu) 
                       - 2.0 * yawspeed_d_enu * omega_d_enu(1) * x_c_enu.dot(z_b_d_enu) - omega_d_enu(0) * omega_d_enu(1) * y_c_enu.dot(y_b_d_enu) 
                       - omega_d_enu(0) * omega_d_enu(2) * y_c_enu.dot(z_b_d_enu);

        Eigen::Vector3d alpha_d_enu = M_inv * b_dot_omega;

        // [5.3] 输出转换：前馈量从 FLU 转换回 PX4 的 FRD
        Eigen::Vector3d omega_d(omega_d_enu.x(), -omega_d_enu.y(), -omega_d_enu.z());
        Eigen::Vector3d alpha_d(alpha_d_enu.x(), -alpha_d_enu.y(), -alpha_d_enu.z());

        // ================= 6. 姿态环：计算控制力矩 tau =================
        // 恢复被你注释掉的完整版：结合 PD 反馈与论文推导的前馈补偿
        Eigen::Vector3d e_w = omega - R.transpose() * R_d * omega_d;

        Eigen::Vector3d tau = -K_R * e_R - K_w * e_w + omega.cross(J * omega) 
                              - J * (omega.cross(R.transpose() * R_d * omega_d) - R.transpose() * R_d * alpha_d);

        // ================= 7. 计算最终推力大小并发布 =================
        double f = -F_des.dot(R.col(2));
        if (f < 0.0) f = 0.0; 

        publish_thrust_and_torque(f, tau, max_thrust, max_torque);
    }

    void publish_thrust_and_torque(double f, const Eigen::Vector3d& tau, double max_f, const Eigen::Vector3d& max_tau)
    {
        uint64_t timestamp = current_odom_.timestamp;

        px4_msgs::msg::VehicleThrustSetpoint thrust_msg{};
        thrust_msg.timestamp = timestamp;
        thrust_msg.xyz[0] = 0.0;
        thrust_msg.xyz[1] = 0.0;
        thrust_msg.xyz[2] = -std::clamp(f / max_f, 0.0, 1.0); 
        thrust_pub_->publish(thrust_msg);

        px4_msgs::msg::VehicleTorqueSetpoint torque_msg{};
        torque_msg.timestamp = timestamp;
        torque_msg.xyz[0] = std::clamp(tau.x() / max_tau.x(), -1.0, 1.0);
        torque_msg.xyz[1] = std::clamp(tau.y() / max_tau.y(), -1.0, 1.0);
        torque_msg.xyz[2] = std::clamp(tau.z() / max_tau.z(), -1.0, 1.0);
        torque_pub_->publish(torque_msg);
    }

    void publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = false;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.thrust_and_torque = true; 
        msg.timestamp = current_odom_.timestamp; 
        offboard_control_mode_pub_->publish(msg);
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = current_odom_.timestamp; 
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