#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;

class GeometricController : public rclcpp::Node
{
public:
    GeometricController() : Node("geometric_controller")
    {

// ================= 1. 声明参数并赋予默认值及描述信息 =================
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        
        // 声明 K_p
        param_desc.description = "Position Proportional Gain [X, Y, Z]";
        this->declare_parameter<std::vector<double>>("K_p", {6.0, 6.5, 8.0}, param_desc);

        // 声明 K_v
        param_desc.description = "Velocity Proportional Gain [X, Y, Z]";
        this->declare_parameter<std::vector<double>>("K_v", {2.0, 2.0, 4.0}, param_desc);

        // 声明 K_i_p
        param_desc.description = "Position Integral Gain [X, Y, Z]";
        this->declare_parameter<std::vector<double>>("K_i_p", {0.2, 0.45, 2.0}, param_desc);

        // 声明 K_i_v
        param_desc.description = "Velocity Integral Gain [X, Y, Z]";
        this->declare_parameter<std::vector<double>>("K_i_v", {0.5, 0.5, 2.0}, param_desc);

        // 🌟 3. 注册参数变动回调函数
        param_callback_handle_ = this->add_on_set_parameters_callback(
                    [this](const std::vector<rclcpp::Parameter> & parameters) {
                        return this->parameters_callback(parameters);
                    });
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

        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(),
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) { 
                current_odom_ = *msg; 
                has_odom_ = true; 
                control_loop(); 
            });
        
        RCLCPP_INFO(this->get_logger(), "Safe Geometric Controller (Dual Integral: Pos + Vel) Started.");
    }

private:
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr att_sp_pub_;
    
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_sub_;

    px4_msgs::msg::VehicleOdometry current_odom_;
    px4_msgs::msg::TrajectorySetpoint current_setpoint_;
    rclcpp::Time last_setpoint_time_;
    
    bool has_odom_ = false;
    bool has_setpoint_ = false;
    uint64_t offboard_setpoint_counter_ = 0;
    
    // 🌟 核心修改 1：独立定义位置误差积分和速度误差积分
    Eigen::Vector3d integral_error_p_{0.0, 0.0, 0.0}; // 位置积分
    Eigen::Vector3d integral_error_v_{0.0, 0.0, 0.0}; // 速度积分
    
    uint64_t last_odom_timestamp_ = 0; 

// 🌟 1. 新增：用于存储动态参数的数组
    std::vector<double> param_K_p_;
    std::vector<double> param_K_v_;
    std::vector<double> param_K_i_p_;
    std::vector<double> param_K_i_v_;

// 🌟 2. 新增：参数回调句柄
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // 🌟 3. 新增：直接在这里写出回调函数的完整实现（带大括号），删掉之前带分号的声明
    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto &param : parameters) {
            if (param.get_name() == "K_p") {
                param_K_p_ = param.as_double_array();
            } else if (param.get_name() == "K_v") {
                param_K_v_ = param.as_double_array();
            } else if (param.get_name() == "K_i_p") {
                param_K_i_p_ = param.as_double_array();
            } else if (param.get_name() == "K_i_v") {
                param_K_i_v_ = param.as_double_array();
            }
        }

        // 打印更新后的 Z 轴参数作为确认
        RCLCPP_INFO(this->get_logger(), "🔧 Gains Updated! K_p[Z]: %.2f, K_v[Z]: %.2f", param_K_p_[2], param_K_v_[2]);
        
        return result;
    }

    void control_loop()
    {
        if (!has_odom_ || !has_setpoint_) return;

        publish_offboard_control_mode();

        if ((this->get_clock()->now() - last_setpoint_time_).seconds() > 0.5) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Setpoint timeout! Danger!");
            
            bool is_grounded = (current_odom_.position[2] > -0.3) &&
                               (std::abs(current_odom_.velocity[0]) < 0.2) &&
                               (std::abs(current_odom_.velocity[1]) < 0.2) &&
                               (std::abs(current_odom_.velocity[2]) < 0.2);

            if (is_grounded) {
                RCLCPP_INFO(this->get_logger(), "Planner offline and drone is grounded. Disarming...");
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
                rclcpp::shutdown(); 
                return;
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Setpoint timeout in Mid-Air!");
                return; 
            }       
        }

        if (offboard_setpoint_counter_ >= 50 && offboard_setpoint_counter_ <= 100) { 
            if (offboard_setpoint_counter_ % 10 == 0) {
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); 
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); 
                if (offboard_setpoint_counter_ == 50) RCLCPP_INFO(this->get_logger(), "Sending Arm commands...");
            }
        }
        if (offboard_setpoint_counter_ < 150) offboard_setpoint_counter_++;

        compute_and_publish_control();
    }


    void compute_and_publish_control()
    {
        const double m = 1.535; 
        const double g = 9.81;  
        const double max_thrust = 28.467; 
        const double max_tilt_angle = 45.0 * M_PI / 180.0; 

        Eigen::Matrix3d D;
        D << 0.25, 0.0,  0.0,
             0.0,  0.25, 0.0,
             0.0,  0.0,  0.0;

// 🌟 将原本写死的硬编码替换为动态参数读取
        Eigen::Matrix3d K_p = Eigen::Vector3d(param_K_p_[0], param_K_p_[1], param_K_p_[2]).asDiagonal();
        Eigen::Matrix3d K_v = Eigen::Vector3d(param_K_v_[0], param_K_v_[1], param_K_v_[2]).asDiagonal();
        
        Eigen::Matrix3d K_i_p = Eigen::Vector3d(param_K_i_p_[0], param_K_i_p_[1], param_K_i_p_[2]).asDiagonal();
        Eigen::Matrix3d K_i_v = Eigen::Vector3d(param_K_i_v_[0], param_K_i_v_[1], param_K_i_v_[2]).asDiagonal();

        // ================= 控制增益 =================
        // Eigen::Matrix3d K_p = Eigen::Vector3d(6.0, 6.5, 8.0).asDiagonal();
        // Eigen::Matrix3d K_v = Eigen::Vector3d(2.0, 2.0, 4.0).asDiagonal();
        
        // // 🌟 核心修改 2：分别配置位置积分增益 (K_i_p) 和速度积分增益 (K_i_v)
        // // 注意：位置积分增益必须非常小，否则极易引起“画马桶圈 (Toilet-bowling)”震荡
        // Eigen::Matrix3d K_i_p = Eigen::Vector3d(0.2, 0.45, 2.0).asDiagonal(); 
        // Eigen::Matrix3d K_i_v = Eigen::Vector3d(0.5, 0.5, 2.0).asDiagonal();

        Eigen::Vector3d p(current_odom_.position[0], current_odom_.position[1], current_odom_.position[2]);
        Eigen::Vector3d v(current_odom_.velocity[0], current_odom_.velocity[1], current_odom_.velocity[2]);
        Eigen::Quaterniond q(current_odom_.q[0], current_odom_.q[1], current_odom_.q[2], current_odom_.q[3]);

        Eigen::Vector3d p_d(current_setpoint_.position[0], current_setpoint_.position[1], current_setpoint_.position[2]);
        Eigen::Vector3d v_d(current_setpoint_.velocity[0], current_setpoint_.velocity[1], current_setpoint_.velocity[2]);
        Eigen::Vector3d a_d(current_setpoint_.acceleration[0], current_setpoint_.acceleration[1], current_setpoint_.acceleration[2]);
        double yaw_d = current_setpoint_.yaw;

        Eigen::Vector3d e_p = p - p_d;
        Eigen::Vector3d e_v = v - v_d;
        
        double dt = 0.01; 
        if (last_odom_timestamp_ != 0 && current_odom_.timestamp > last_odom_timestamp_) {
            dt = static_cast<double>(current_odom_.timestamp - last_odom_timestamp_) / 1e6; 
        }
        last_odom_timestamp_ = current_odom_.timestamp;
        if (dt > 0.05) dt = 0.01; 

        // 🌟 核心修改 3：双通道积分与严格的防饱和限幅
        
        // 1. 位置误差积分 (消除绝对稳态误差)
        integral_error_p_ += e_p * dt;
        // 位置积分限幅非常严格 (水平最多补偿 1.0 m/s^2 的加速度)
        integral_error_p_.x() = std::clamp(integral_error_p_.x(), -1.0, 1.0);
        integral_error_p_.y() = std::clamp(integral_error_p_.y(), -1.0, 1.0);
        integral_error_p_.z() = std::clamp(integral_error_p_.z(), -2.0, 2.0);

        // 2. 速度误差积分 (估计恒定扰动)
        integral_error_v_ += e_v * dt;
        integral_error_v_.x() = std::clamp(integral_error_v_.x(), -2.0, 2.0);
        integral_error_v_.y() = std::clamp(integral_error_v_.y(), -2.0, 2.0);
        integral_error_v_.z() = std::clamp(integral_error_v_.z(), -3.0, 3.0); 

        // 🌟 核心修改 4：将两套积分补偿全部融合进期望加速度
        Eigen::Vector3d a_cmd = a_d - K_p * e_p - K_v * e_v - K_i_p * integral_error_p_ - K_i_v * integral_error_v_;

        // 基础推力、坐标转换与阻力补偿
        Eigen::Vector3d e_3(0.0, 0.0, 1.0);
        Eigen::Vector3d T_vec_pre = m * (g * e_3 - a_cmd);

        Eigen::Vector3d z_b_pre = T_vec_pre.normalized();
        Eigen::Vector3d x_c(std::cos(yaw_d), std::sin(yaw_d), 0.0);
        Eigen::Vector3d y_b_pre = z_b_pre.cross(x_c).normalized();
        Eigen::Vector3d x_b_pre = y_b_pre.cross(z_b_pre);
        Eigen::Matrix3d R_pre;
        R_pre << x_b_pre, y_b_pre, z_b_pre;

        Eigen::Vector3d a_drag = R_pre * D * R_pre.transpose() * v_d;
        
        Eigen::Vector3d T_vec = m * (g * e_3 - a_cmd - a_drag);

        // 输出钳制
        if (T_vec.z() < 0.1) T_vec.z() = 0.1; 
        
        double current_tilt_cos = std::abs(T_vec.z()) / T_vec.norm();
        if (current_tilt_cos < std::cos(max_tilt_angle)) {
            double max_xy_mag = std::abs(T_vec.z()) * std::tan(max_tilt_angle);
            double xy_mag = std::sqrt(T_vec.x() * T_vec.x() + T_vec.y() * T_vec.y());
            T_vec.x() *= max_xy_mag / xy_mag;
            T_vec.y() *= max_xy_mag / xy_mag;
        }

        double T_norm = T_vec.norm();

        // 几何映射
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
        px4_msgs::msg::VehicleAttitudeSetpoint msg{};
        msg.timestamp = current_odom_.timestamp;
        
        msg.q_d[0] = q_d.w();
        msg.q_d[1] = q_d.x();
        msg.q_d[2] = q_d.y();
        msg.q_d[3] = q_d.z();
        
        double normalized_thrust = std::clamp(T_norm / max_thrust, 0.05, 0.95);

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