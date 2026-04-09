#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
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
        rates_pub_                 = this->create_publisher<px4_msgs::msg::VehicleRatesSetpoint>("/fmu/in/vehicle_rates_setpoint", 10);

        // --- Subscribers ---
        setpoint_sub_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
            "/geometric_controller/setpoint", 10,
            [this](const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) { 
                current_setpoint_ = *msg; 
                has_setpoint_ = true; 
                last_setpoint_time_ = this->get_clock()->now(); // 🌟 记录最后一次收到轨迹的时间
            });

        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(),
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) { 
                current_odom_ = *msg; 
                has_odom_ = true; 
            });
        
        // 250Hz 控制循环
        timer_ = this->create_wall_timer(4ms, std::bind(&GeometricController::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Geometric Controller (Body Rate) with Failsafe Started.");
    }

private:
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rates_pub_;
    
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_sub_;

    px4_msgs::msg::VehicleOdometry current_odom_;
    px4_msgs::msg::TrajectorySetpoint current_setpoint_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    bool has_odom_ = false;
    bool has_setpoint_ = false;
    uint64_t offboard_setpoint_counter_ = 0;
    rclcpp::Time last_setpoint_time_; // 🌟 超时检测时钟
    
    double integral_error_z_ = 0.0;

    Eigen::Vector3d vee_map(const Eigen::Matrix3d& S) const {
        return Eigen::Vector3d(S(2, 1), S(0, 2), S(1, 0));
    }

    void control_loop()
    {
        if (!has_odom_) return;

        // 🌟 安全看门狗逻辑 (Failsafe)
        if (has_setpoint_) {
            double time_since_last_setpoint = (this->get_clock()->now() - last_setpoint_time_).seconds();
            if (time_since_last_setpoint > 0.5) { // 超过 0.5s 没收到新轨迹
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "⚠️ Setpoint Timeout! Trajectory Planner Lost!");
                
                // 判断是否在地面 (NED 坐标系下，Z > -0.5 且速度极小)
                bool is_grounded = (current_odom_.position[2] > -0.5) &&
                                   (std::abs(current_odom_.velocity[0]) < 0.2) &&
                                   (std::abs(current_odom_.velocity[1]) < 0.2) &&
                                   (std::abs(current_odom_.velocity[2]) < 0.2);

                if (is_grounded) {
                    RCLCPP_INFO(this->get_logger(), "Drone is grounded. Disarming and shutting down.");
                    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0); // Disarm
                    rclcpp::shutdown(); 
                } else {
                    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Mid-air connection lost! Switching to HOLD/LOITER mode.");
                    // 🌟 空中丢失信号，切回安全的 Hold/Loiter 模式 (Auto mode: 4, Loiter: 3)
                    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 3); 
                }
                return; // 拦截计算，不再发无效指令
            }
        } else {
            return; // 启动初期还没收到第一条轨迹时，保持静默
        }

        // ================= 正常控制流 =================
        publish_offboard_control_mode();

        if (offboard_setpoint_counter_ == 125) { 
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // Offboard
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); // Arm
            RCLCPP_INFO(this->get_logger(), "Arming and setting Offboard mode!");
        }
        if (offboard_setpoint_counter_ < 250) offboard_setpoint_counter_++;

        compute_and_publish_control();
    }

    void compute_and_publish_control()
    {
        // ================= 物理参数 =================
        const double m = 1.535; // kg
        const double g = 9.81;  // m/s^2
        const double max_thrust = 28.467; // N

        Eigen::Matrix3d D;
        D << 0.15, 0.0,  0.0,
             0.0,  0.15, 0.0,
             0.0,  0.0,  0.0;

        // ================= 控制增益 =================
        Eigen::Matrix3d K_p = Eigen::Vector3d(3.5, 3.5, 8.0).asDiagonal();
        Eigen::Matrix3d K_v = Eigen::Vector3d(2.0, 2.0, 4.0).asDiagonal();
        double K_i_z = 2.0;

        Eigen::Matrix3d K_R = Eigen::Vector3d(6.0, 6.0, 3.0).asDiagonal();

        // ================= 1. 提取当前状态 =================
        Eigen::Vector3d p(current_odom_.position[0], current_odom_.position[1], current_odom_.position[2]);
        Eigen::Vector3d v(current_odom_.velocity[0], current_odom_.velocity[1], current_odom_.velocity[2]);
        Eigen::Quaterniond q(current_odom_.q[0], current_odom_.q[1], current_odom_.q[2], current_odom_.q[3]);
        Eigen::Matrix3d R = q.toRotationMatrix();

        // ================= 2. 提取期望状态 =================
        Eigen::Vector3d p_d(current_setpoint_.position[0], current_setpoint_.position[1], current_setpoint_.position[2]);
        Eigen::Vector3d v_d(current_setpoint_.velocity[0], current_setpoint_.velocity[1], current_setpoint_.velocity[2]);
        Eigen::Vector3d a_d(current_setpoint_.acceleration[0], current_setpoint_.acceleration[1], current_setpoint_.acceleration[2]);
        Eigen::Vector3d j_d(current_setpoint_.jerk[0], current_setpoint_.jerk[1], current_setpoint_.jerk[2]); 
        
        double yaw_d = current_setpoint_.yaw;
        double yawspeed_d = current_setpoint_.yawspeed;

        // ================= 3. 位置环：计算期望推力 =================
        Eigen::Vector3d e_p = p - p_d;
        Eigen::Vector3d e_v = v - v_d;
        
        integral_error_z_ += e_p.z() * 0.004; 
        integral_error_z_ = std::clamp(integral_error_z_, -6.0, 6.0);
    
        Eigen::Vector3d F_des_pre = -K_p * e_p - K_v * e_v + m * a_d - Eigen::Vector3d(0, 0, m * g);
        Eigen::Vector3d z_b_d_pre = (-F_des_pre).normalized();
        Eigen::Vector3d x_c(std::cos(yaw_d), std::sin(yaw_d), 0.0);
        Eigen::Vector3d y_b_d_pre = z_b_d_pre.cross(x_c).normalized();
        Eigen::Vector3d x_b_d_pre = y_b_d_pre.cross(z_b_d_pre);
        Eigen::Matrix3d R_d_pre;
        R_d_pre << x_b_d_pre, y_b_d_pre, z_b_d_pre;

        Eigen::Vector3d F_drag = m * R_d_pre * D * R_d_pre.transpose() * v_d;
        
        Eigen::Vector3d F_des = -K_p * e_p - K_v * e_v + m * a_d - Eigen::Vector3d(0, 0, m * g) + F_drag;
        F_des.z() -= K_i_z * integral_error_z_;

        Eigen::Vector3d T_vec = -F_des; 
        double T_norm = T_vec.norm();

        // ================= 4. 几何映射：最终期望姿态 =================
        Eigen::Vector3d z_b_d = T_vec.normalized(); 
        Eigen::Vector3d y_c(-std::sin(yaw_d), std::cos(yaw_d), 0.0);
        Eigen::Vector3d y_b_d = z_b_d.cross(x_c).normalized();
        Eigen::Vector3d x_b_d = y_b_d.cross(z_b_d);
        
        Eigen::Matrix3d R_d;
        R_d << x_b_d, y_b_d, z_b_d; 

        Eigen::Matrix3d err_matrix = 0.5 * (R_d.transpose() * R - R.transpose() * R_d);
        Eigen::Vector3d e_R = vee_map(err_matrix);

        // ================= 5. 微分平坦性：NED-ENU-NED 包装器 =================
        Eigen::Vector3d v_d_enu(v_d.x(), -v_d.y(), -v_d.z());
        Eigen::Vector3d a_d_enu(a_d.x(), -a_d.y(), -a_d.z());
        Eigen::Vector3d j_d_enu(j_d.x(), -j_d.y(), -j_d.z());
        double yaw_d_enu = -yaw_d;
        double yawspeed_d_enu = -yawspeed_d;

        Eigen::Vector3d z_b_d_enu(z_b_d.x(), -z_b_d.y(), -z_b_d.z());
        Eigen::Vector3d x_c_enu(std::cos(yaw_d_enu), std::sin(yaw_d_enu), 0.0);
        Eigen::Vector3d y_c_enu(-std::sin(yaw_d_enu), std::cos(yaw_d_enu), 0.0);
        
        Eigen::Vector3d y_b_d_enu = z_b_d_enu.cross(x_c_enu).normalized();
        Eigen::Vector3d x_b_d_enu = y_b_d_enu.cross(z_b_d_enu);

        double d_x = D(0,0); double d_y = D(1,1); double d_z = D(2,2);
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

        Eigen::Vector3d b_omega;
        b_omega(0) = j_x + d_x * a_x;
        b_omega(1) = -j_y - d_y * a_y;
        b_omega(2) = yawspeed_d_enu * x_c_enu.dot(x_b_d_enu);

        Eigen::Vector3d omega_d_enu = M_omega.inverse() * b_omega;
        Eigen::Vector3d omega_d(omega_d_enu.x(), -omega_d_enu.y(), -omega_d_enu.z());

        // ================= 6. 姿态环：生成角速度指令 =================
        Eigen::Vector3d omega_ff = R.transpose() * R_d * omega_d;
        Eigen::Vector3d omega_cmd = omega_ff - K_R * e_R;

        // ================= 7. 计算最终推力大小并发布 =================
        double f = -F_des.dot(R.col(2));
        if (f < 0.0) f = 0.0; 

        publish_rates_and_thrust(f, omega_cmd, max_thrust);
    }

    void publish_rates_and_thrust(double f, const Eigen::Vector3d& omega_cmd, double max_f)
    {
        uint64_t timestamp = current_odom_.timestamp;

        px4_msgs::msg::VehicleRatesSetpoint msg{};
        msg.timestamp = timestamp;
        
        msg.roll = omega_cmd.x();
        msg.pitch = omega_cmd.y();
        msg.yaw = omega_cmd.z();
        
        msg.thrust_body[0] = 0.0;
        msg.thrust_body[1] = 0.0;
        msg.thrust_body[2] = -std::clamp(f / max_f, 0.0, 1.0); 
        
        rates_pub_->publish(msg);
    }

    void publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = false;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = true;  
        msg.thrust_and_torque = false; 
        msg.timestamp = current_odom_.timestamp; 
        offboard_control_mode_pub_->publish(msg);
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.param3 = param3; // 支持多参数 (用于切模式)
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