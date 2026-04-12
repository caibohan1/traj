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

// 🌟 0. 定义一个生成参数描述符（限制范围）的辅助函数
    auto make_float_desc = [](std::string description, double min, double max, double step) {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.description = description;
        // 🌟🌟🌟 关键修复：显式告诉 ROS 2 这是一个双精度浮点数
        desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;

        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = min;
        range.to_value = max;
        range.step = step;
        desc.floating_point_range.push_back(range);
        return desc;
    };

// 🌟 1. 拆分数组，声明独立的标量参数
        this->declare_parameter<double>("K_p_x", 6.0, make_float_desc("Proportional Gain P_X", 0.0, 15.0, 0.1));
        this->declare_parameter<double>("K_p_y", 6.5, make_float_desc("Proportional Gain P_Y", 0.0, 15.0, 0.1));
        this->declare_parameter<double>("K_p_z", 8.0, make_float_desc("Proportional Gain P_Z", 0.0, 15.0, 0.1));

        this->declare_parameter<double>("K_v_x", 2.0, make_float_desc("Derivative Gain V_X", 0.0, 10.0, 0.1));
        this->declare_parameter<double>("K_v_y", 2.0, make_float_desc("Derivative Gain V_Y", 0.0, 10.0, 0.1));
        this->declare_parameter<double>("K_v_z", 4.0, make_float_desc("Derivative Gain V_Z", 0.0, 10.0, 0.1));

        this->declare_parameter<double>("K_i_p_x", 0.2, make_float_desc("Integral Gain P_X", 0.0, 5.0, 0.1));
        this->declare_parameter<double>("K_i_p_y", 0.45, make_float_desc("Integral Gain P_Y", 0.0, 5.0, 0.05));
        this->declare_parameter<double>("K_i_p_z", 2.0, make_float_desc("Integral Gain P_Z", 0.0, 5.0, 0.1));

        this->declare_parameter<double>("K_i_v_x", 0.5, make_float_desc("Integral Gain V_X", 0.0, 5.0, 0.1));
        this->declare_parameter<double>("K_i_v_y", 0.5, make_float_desc("Integral Gain V_Y", 0.0, 5.0, 0.1));
        this->declare_parameter<double>("K_i_v_z", 2.0, make_float_desc("Integral Gain V_Z", 0.0, 5.0, 0.1));
        // 🌟 2. 初始化 Eigen 矩阵
        K_p_ = Eigen::Vector3d(this->get_parameter("K_p_x").as_double(),
                               this->get_parameter("K_p_y").as_double(),
                               this->get_parameter("K_p_z").as_double()).asDiagonal();

        K_v_ = Eigen::Vector3d(this->get_parameter("K_v_x").as_double(),
                               this->get_parameter("K_v_y").as_double(),
                               this->get_parameter("K_v_z").as_double()).asDiagonal();

        K_i_p_ = Eigen::Vector3d(this->get_parameter("K_i_p_x").as_double(),
                                 this->get_parameter("K_i_p_y").as_double(),
                                 this->get_parameter("K_i_p_z").as_double()).asDiagonal();

        K_i_v_ = Eigen::Vector3d(this->get_parameter("K_i_v_x").as_double(),
                                 this->get_parameter("K_i_v_y").as_double(),
                                 this->get_parameter("K_i_v_z").as_double()).asDiagonal();

        // 🌟 3. 注册参数动态回调
        param_subscriber_ = this->add_on_set_parameters_callback(
            std::bind(&GeometricController::parametersCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Dynamic Params Loaded! K_p_z: %.2f", K_p_(2, 2));
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
                control_loop(); // 收到最新里程计后触发控制计算
            });
        
        RCLCPP_INFO(this->get_logger(), "Safe Geometric Controller (NED Fixed) with Dual Integral Started.");
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
    
    // 🌟 核心修改 1：独立定义三轴位置误差积分和速度误差积分
    Eigen::Vector3d integral_error_p_{0.0, 0.0, 0.0}; 
    Eigen::Vector3d integral_error_v_{0.0, 0.0, 0.0}; 
    
    uint64_t last_odom_timestamp_ = 0; 
// 🌟 1. 新增：直接把四个增益矩阵作为类的成员变量存起来
    Eigen::Matrix3d K_p_;
    Eigen::Matrix3d K_v_;
    Eigen::Matrix3d K_i_p_;
    Eigen::Matrix3d K_i_v_;

// 参数回调句柄
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_subscriber_;

// 🌟 4. 参数回调处理函数：当你在 rqt 中拖动滑块时，这里会被触发
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto &param : parameters) {
            // 解析 K_p
            if (param.get_name() == "K_p_x") K_p_(0, 0) = param.as_double();
            else if (param.get_name() == "K_p_y") K_p_(1, 1) = param.as_double();
            else if (param.get_name() == "K_p_z") K_p_(2, 2) = param.as_double();
            
            // 解析 K_v
            else if (param.get_name() == "K_v_x") K_v_(0, 0) = param.as_double();
            else if (param.get_name() == "K_v_y") K_v_(1, 1) = param.as_double();
            else if (param.get_name() == "K_v_z") K_v_(2, 2) = param.as_double();

            // 解析 K_i_p
            else if (param.get_name() == "K_i_p_x") K_i_p_(0, 0) = param.as_double();
            else if (param.get_name() == "K_i_p_y") K_i_p_(1, 1) = param.as_double();
            else if (param.get_name() == "K_i_p_z") K_i_p_(2, 2) = param.as_double();

            // 解析 K_i_v
            else if (param.get_name() == "K_i_v_x") K_i_v_(0, 0) = param.as_double();
            else if (param.get_name() == "K_i_v_y") K_i_v_(1, 1) = param.as_double();
            else if (param.get_name() == "K_i_v_z") K_i_v_(2, 2) = param.as_double();
        }

        RCLCPP_INFO(this->get_logger(), "Parameters updated dynamically!");
        return result;
    }

    void control_loop()
    {
        if (!has_odom_ || !has_setpoint_) return;

        publish_offboard_control_mode();

        // 超时保护：超过 0.5s 没收到轨迹
        if ((this->get_clock()->now() - last_setpoint_time_).seconds() > 0.5) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Setpoint timeout! Danger!");
            
            // NED 坐标系下，Z > -0.3 代表高度低于 0.3 米；同时三轴速度极小
            bool is_grounded = (current_odom_.position[2] > -0.3) &&
                               (std::abs(current_odom_.velocity[0]) < 0.2) &&
                               (std::abs(current_odom_.velocity[1]) < 0.2) &&
                               (std::abs(current_odom_.velocity[2]) < 0.2);

            if (is_grounded) {
                RCLCPP_INFO(this->get_logger(), "Planner offline and drone is grounded. Disarming and shutting down...");
                
                // 发送安全上锁 (Disarm) 指令
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
                
                rclcpp::shutdown(); 
                return;
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Setpoint timeout in Mid-Air! Danger!");
                return; // 防止僵尸节点继续发老指令
            }       
        }

        // 强化解锁逻辑：在 50 到 100 周期之间发切模式和解锁指令
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
        const double m = 1.535; 
        const double g = 9.81;  
        const double max_thrust = 25.1; 
        const double max_tilt_angle = 45.0 * M_PI / 180.0; 

        // 线性空气阻力系数矩阵
        Eigen::Matrix3d D;
        D << 0.25, 0.0,  0.0,
             0.0,  0.25, 0.0,
             0.0,  0.0,  0.0;

        // ================= 🌟 核心修改 2：双积分控制增益 =================
        // Eigen::Matrix3d K_p = Eigen::Vector3d(6.0, 6.5, 8.0).asDiagonal();
        // Eigen::Matrix3d K_v = Eigen::Vector3d(2.0, 2.0, 4.0).asDiagonal();
        
        // // 分别配置位置积分增益 (K_i_p) 和速度积分增益 (K_i_v)
        // Eigen::Matrix3d K_i_p = Eigen::Vector3d(0.2, 0.45, 2.0).asDiagonal(); 
        // Eigen::Matrix3d K_i_v = Eigen::Vector3d(0.5, 0.5, 2.0).asDiagonal();

        // ================= 1. 提取当前状态 (NED) =================
        Eigen::Vector3d p(current_odom_.position[0], current_odom_.position[1], current_odom_.position[2]);
        Eigen::Vector3d v(current_odom_.velocity[0], current_odom_.velocity[1], current_odom_.velocity[2]);
        Eigen::Quaterniond q(current_odom_.q[0], current_odom_.q[1], current_odom_.q[2], current_odom_.q[3]);

        // ================= 2. 提取期望状态 (NED) =================
        Eigen::Vector3d p_d(current_setpoint_.position[0], current_setpoint_.position[1], current_setpoint_.position[2]);
        Eigen::Vector3d v_d(current_setpoint_.velocity[0], current_setpoint_.velocity[1], current_setpoint_.velocity[2]);
        Eigen::Vector3d a_d(current_setpoint_.acceleration[0], current_setpoint_.acceleration[1], current_setpoint_.acceleration[2]);
        double yaw_d = current_setpoint_.yaw;

        // ================= 3. 误差计算与积分 =================
        Eigen::Vector3d e_p = p - p_d;
        Eigen::Vector3d e_v = v - v_d;
        
        double dt = 0.01; 
        if (last_odom_timestamp_ != 0 && current_odom_.timestamp > last_odom_timestamp_) {
            dt = static_cast<double>(current_odom_.timestamp - last_odom_timestamp_) / 1e6; 
        }
        last_odom_timestamp_ = current_odom_.timestamp;
        if (dt > 0.05 || dt <= 0.0) dt = 0.01; 

        // 🌟 核心修改 3：双通道积分与严格防饱和限幅
        // 1. 位置误差积分
        integral_error_p_ += e_p * dt;
        integral_error_p_.x() = std::clamp(integral_error_p_.x(), -1.0, 1.0);
        integral_error_p_.y() = std::clamp(integral_error_p_.y(), -1.0, 1.0);
        integral_error_p_.z() = std::clamp(integral_error_p_.z(), -2.0, 2.0);

        // 2. 速度误差积分
        integral_error_v_ += e_v * dt;
        integral_error_v_.x() = std::clamp(integral_error_v_.x(), -2.0, 2.0);
        integral_error_v_.y() = std::clamp(integral_error_v_.y(), -2.0, 2.0);
        integral_error_v_.z() = std::clamp(integral_error_v_.z(), -3.0, 3.0); 

        // 🌟 核心修改 4：期望加速度 (包含 PD 反馈、前馈、及所有积分项)
        Eigen::Vector3d a_cmd = a_d - K_p_ * e_p - K_v_ * e_v - K_i_p_ * integral_error_p_ - K_i_v_ * integral_error_v_;

        // 基础期望推力矢量 (无阻力)：T_vec = m * (g * e_3 - a_cmd) 
        // NED 坐标系重力沿 Z 轴正方向 (+g)
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
        // 防止推力反向 (NED坐标系中，机体Z轴应当保持朝下，所以T_vec.z()必须为正数)
        if (T_vec.z() < 0.1) T_vec.z() = 0.1; 
        
        // 最大倾角圆锥钳制
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

        // 打印期望姿态四元数和归一化推力（每 500 毫秒打印一次）
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            500,
            "📐 Att Setpoint -> q: [%.2f, %.2f, %.2f, %.2f] | Thrust: %.2f (Norm: %.2f)",
            q_d.w(), q_d.x(), q_d.y(), q_d.z(), T_norm, normalized_thrust
        );
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