#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp> // 新增：Offboard 心跳包
#include <px4_msgs/msg/vehicle_command.hpp>       // 新增：指令发送
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <nav_msgs/msg/path.hpp>                 
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class Px4AutoFigure8 : public rclcpp::Node
{
public:
    Px4AutoFigure8() : Node("px4_auto_figure8")
    {
        // ================= 发布者 =================
        // 🌟 发布到 PX4 官方设定点话题，由 PX4 外环直接跟踪
        setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
            
        // 🌟 发布 Offboard 控制模式心跳包
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
            
        // 🌟 发布系统指令（解锁、切模式）
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
                    // 🌟 首次收到里程计，立刻锁定当前物理坐标作为相对起点！
                    start_x_ = msg->position[0];
                    start_y_ = msg->position[1];
                    start_z_ = msg->position[2];
                    double q_w = msg->q[0], q_x = msg->q[1], q_y = msg->q[2], q_z = msg->q[3];
                    start_yaw_ = std::atan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z));
                    has_odom_ = true;
                    
                    RCLCPP_INFO(this->get_logger(), "📍 Initial Pose Locked! X:%.2f, Y:%.2f, Yaw:%.2f rad", start_x_, start_y_, start_yaw_);
                    
                    // 锁定起点后生成 RViz 视觉轨迹
                    init_visual_path();
                }
            });

        time_step_ = 0;
        offboard_setpoint_counter_ = 0;

        // 100Hz 主循环
        timer_ = this->create_wall_timer(
            10ms, std::bind(&Px4AutoFigure8::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "🚀 Auto-Takeoff Figure-8 Node Started. Connecting to PX4...");
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

    void init_visual_path()
    {
        double A = 2.0;  
        double B = 1.0;  
        double omega_max = 0.25; 
        double target_z = -1.8; 
        
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

    // 🌟 发送 Offboard 心跳包
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

    // 🌟 发送底层系统指令（切模式、解锁）
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
        if (!has_odom_) return; // 没收到真实位置前不发送指令

        // 1. 必须始终发送 Offboard 心跳包 (100Hz)
        publish_offboard_control_mode();

        px4_msgs::msg::TrajectorySetpoint msg{};
        auto timestamp = this->get_clock()->now();
        msg.timestamp = timestamp.nanoseconds() / 1000;

        // ================= 状态机：自动解锁与切模式 =================
        if (!trajectory_started_) {
            // 起飞预热：发送固定原点设定点
            msg.position = {(float)start_x_, (float)start_y_, (float)start_z_};
            msg.yaw = (float)start_yaw_;
            msg.velocity = {0.0, 0.0, 0.0};
            msg.acceleration = {0.0, 0.0, 0.0};
            msg.yawspeed = 0.0;
            setpoint_pub_->publish(msg);

            // 预热 1 秒后 (100次循环)，触发解锁和 Offboard
            if (offboard_setpoint_counter_ == 100) {
                RCLCPP_INFO(this->get_logger(), "Sending Offboard & Arm commands...");
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
            }

            // 重试机制：防止丢包
            if (offboard_setpoint_counter_ > 100 && offboard_setpoint_counter_ % 100 == 0) {
                if (!is_offboard_) publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                if (!is_armed_) publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
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
        const double target_z = -1.8;     
        
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
        
        // 🌟 核心：检测到落地完毕后，自动上锁并关闭节点
        if (t > T_total + T_wait_after_land) {
            RCLCPP_INFO(this->get_logger(), "🏁 Mission Accomplished. Disarming and shutting down...");
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0); // 上锁
            rclcpp::shutdown(); 
            return; 
        }

        double px=0, py=0, pz=0;
        double vz=0, az=0;
        double theta=0, omega=0, alpha=0;

        // 五段式运动学推导保持不变
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

        // ================= 严格解析：广义 8 字形参数映射 =================
        double c1 = std::cos(theta);
        double s1 = std::sin(theta);
        double c2 = std::cos(2.0 * theta);
        double s2 = std::sin(2.0 * theta);

        // 位置 (叠加起点偏移)
        px = start_x_ + A * s1;
        py = start_y_ + B * s2;
        pz = start_z_ + pz;
        
        // 速度
        double vx = A * omega * c1;
        double vy = 2.0 * B * omega * c2;
        
        // 加速度 (作为强效前馈发送给 PX4 外环)
        double ax = A * (alpha * c1 - std::pow(omega, 2) * s1);
        double ay = 2.0 * B * (alpha * c2 - 2.0 * std::pow(omega, 2) * s2);

        msg.position = {(float)px, (float)py, (float)pz};
        msg.velocity = {(float)vx, (float)vy, (float)vz};
        msg.acceleration = {(float)ax, (float)ay, (float)az};

        // ================= 偏航角始终锁定为起飞朝向 =================
        msg.yaw = (float)start_yaw_; 
        msg.yawspeed = 0.0;          

        // ================= 发送数据 =================
        setpoint_pub_->publish(msg);

        expected_path_.header.stamp = timestamp;
        path_pub_->publish(expected_path_);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), 
            *this->get_clock(), 
            500, 
            "🎯 PX4 Tracking -> X: % .3f, Y: % .3f, Z: % .3f | Mode Time: % .2f s", 
            px, py, pz, t
        );

        time_step_++;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4AutoFigure8>());
    rclcpp::shutdown();
    return 0;
}