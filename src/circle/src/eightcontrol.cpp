#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <nav_msgs/msg/path.hpp>                 
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <chrono>
#include <cmath>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

using namespace std::chrono_literals;

class TrajectoryPlanner : public rclcpp::Node
{
public:
    TrajectoryPlanner() : Node("trajectory_planner")
    {
        setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/geometric_controller/setpoint", 10);

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/geometric_controller/expected_path", 10);

        // ================= 监听飞控底层状态 =================
        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", 
            rclcpp::QoS(10).best_effort(), 
            [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
                
                bool is_armed = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
                bool is_offboard = (msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);

                if (is_armed && is_offboard && !trajectory_started_) {
                    if (has_odom_) {
                        // 🌟 核心：在切入 Offboard 瞬间，锁定当前物理坐标作为相对起点！
                        start_x_ = current_odom_.position[0];
                        start_y_ = current_odom_.position[1];
                        start_z_ = current_odom_.position[2];
                        start_yaw_ = current_yaw_;

                        RCLCPP_INFO(this->get_logger(), "📍 Initial Pose Locked! X:%.2f, Y:%.2f, Yaw:%.2f", start_x_, start_y_, start_yaw_);
                        RCLCPP_INFO(this->get_logger(), "🚀 Offboard & Armed confirmed! Taking off...");
                        trajectory_started_ = true;
                        
                        // 锁定起点后再生成 RViz 视觉轨迹
                        init_visual_path();
                    }
                }
            });

        // ================= 订阅真实里程计 =================
        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(),
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                current_odom_ = *msg;
                has_odom_ = true;
                double q_w = msg->q[0], q_x = msg->q[1], q_y = msg->q[2], q_z = msg->q[3];
                current_yaw_ = std::atan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z));
            });

        time_step_ = 0;

        timer_ = this->create_wall_timer(
            10ms, std::bind(&TrajectoryPlanner::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Clean Decoupled Figure-8 Planner (High-Speed) Started.");
    }

private:
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; 
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    
    px4_msgs::msg::VehicleOdometry current_odom_;
    bool has_odom_ = false;

    double current_yaw_ = 0.0;
    double start_x_ = 0.0, start_y_ = 0.0, start_z_ = 0.0, start_yaw_ = 0.0;

    bool trajectory_started_ = false;

    nav_msgs::msg::Path expected_path_; 
    uint64_t time_step_;

    void init_visual_path()
    {
        double A = 2.0;  
        double B = 1.0;  
        double omega_max = 0.25; 
        double target_z = -1.8; // NED高度为负
        
        expected_path_.header.frame_id = "map"; 
        expected_path_.poses.clear();
        
        // 绘制完整 8 字形参考轨迹 (RViz)
        double T = 2.0 * M_PI / omega_max; 
        for (double t = 0; t <= T; t += 0.05) {
            geometry_msgs::msg::PoseStamped pose;
            // 叠加相对偏移量
            pose.pose.position.x = start_x_ + A * std::sin(omega_max * t);
            pose.pose.position.y = start_y_ + B * std::sin(2.0 * omega_max * t);
            pose.pose.position.z = start_z_ + target_z; 
            expected_path_.poses.push_back(pose);
        }
    }

    void timer_callback()
    {
        px4_msgs::msg::TrajectorySetpoint msg{};

        // ================= 时间冻结与相对位置预热逻辑 =================
        if (!trajectory_started_) {
            time_step_ = 0; 
            
            // 发布当前真实物理位置（相对零点），让误差强制为0
            if (has_odom_) {
                msg.position = {current_odom_.position[0], current_odom_.position[1], current_odom_.position[2]};
                msg.yaw = (float)current_yaw_;
            } else {
                msg.position = {0.0, 0.0, 0.0};
                msg.yaw = 0.0;
            }
            msg.velocity = {0.0, 0.0, 0.0};
            msg.acceleration = {0.0, 0.0, 0.0};
            msg.yawspeed = 0.0;

            auto timestamp = this->get_clock()->now();
            msg.timestamp = timestamp.nanoseconds() / 1000;
            setpoint_pub_->publish(msg);

            static int wait_print_counter = 0;
            if (wait_print_counter++ % 200 == 0) {
                RCLCPP_INFO(this->get_logger(), "Waiting for Arm & Offboard... Pre-heating controller.");
            }
            return; // 拦截后续计算
        }

        double t = static_cast<double>(time_step_) * 0.01; 
        
        // ================= 核心物理参数 =================
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
        
        if (t > T_total + T_wait_after_land) {
            RCLCPP_INFO(this->get_logger(), "Mission Accomplished. Shutting down Planner...");
            rclcpp::shutdown(); 
            return; 
        }

        double px=0, py=0, pz=0;
        double vz=0, az=0;
        double theta=0, omega=0, alpha=0;

        // ================= 解耦状态机推导 =================
        if (t < T_takeoff) {
            double tau = t / T_takeoff;
            double tau2 = tau * tau; double tau3 = tau2 * tau;
            double tau4 = tau3 * tau; double tau5 = tau4 * tau;

            double S = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
            double dS = (1.0 / T_takeoff) * (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4);
            double ddS = (1.0 / std::pow(T_takeoff, 2)) * (60.0 * tau - 180.0 * tau2 + 120.0 * tau3);

            pz = target_z * S;
            vz = target_z * dS; az = target_z * ddS; 
            
            omega = 0; alpha = 0; theta = 0;
        } 
        else if (t < T_takeoff + T_accel) {
            double t_loc = t - T_takeoff;
            double tau = t_loc / T_accel;
            double tau2 = tau * tau; double tau3 = tau2 * tau;
            double tau4 = tau3 * tau; double tau5 = tau4 * tau;
            double tau6 = tau5 * tau;

            double S = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
            double dS = (1.0 / T_accel) * (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4);

            pz = target_z; vz = 0; az = 0; 

            omega = omega_max * S;
            alpha = omega_max * dS; 

            double I_S = 2.5 * tau4 - 3.0 * tau5 + tau6;
            theta = omega_max * T_accel * I_S; 
        } 
        else if (t < T_takeoff + T_accel + T_cruise) {
            double t_loc = t - T_takeoff - T_accel;
            pz = target_z; vz = 0; az = 0; 
            omega = omega_max; alpha = 0; 
            theta = (circles_accel * 2.0 * M_PI) + (omega_max * t_loc);
        } 
        else if (t < T_takeoff + T_accel + T_cruise + T_decel) {
            double t_loc = t - T_takeoff - T_accel - T_cruise;
            double tau = t_loc / T_decel;
            double tau2 = tau * tau; double tau3 = tau2 * tau;
            double tau4 = tau3 * tau; double tau5 = tau4 * tau;
            double tau6 = tau5 * tau;

            double S = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
            double dS = (1.0 / T_decel) * (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4);

            pz = target_z; vz = 0; az = 0; 

            omega = omega_max * (1.0 - S);
            alpha = -omega_max * dS; 

            double I_1_minus_S = tau - (2.5 * tau4 - 3.0 * tau5 + tau6);
            theta = (circles_accel + circles_cruise) * 2.0 * M_PI + (omega_max * T_decel * I_1_minus_S); 
        } 
        else if (t < T_total) {
            double t_loc = t - T_takeoff - T_accel - T_cruise - T_decel;
            double tau = t_loc / T_land;
            double tau2 = tau * tau; double tau3 = tau2 * tau;
            double tau4 = tau3 * tau; double tau5 = tau4 * tau;

            double S = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
            double dS = (1.0 / T_land) * (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4);
            double ddS = (1.0 / std::pow(T_land, 2)) * (60.0 * tau - 180.0 * tau2 + 120.0 * tau3);

            pz = target_z * (1.0 - S);
            vz = -target_z * dS; az = -target_z * ddS; 

            omega = 0; alpha = 0; 
            theta = (circles_accel + circles_cruise + circles_decel) * 2.0 * M_PI; 
        }
        else {
            pz = 0; vz = 0; az = 0; 
            omega = 0; alpha = 0; 
            theta = (circles_accel + circles_cruise + circles_decel) * 2.0 * M_PI;
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
        
        // 加速度
        double ax = A * (alpha * c1 - std::pow(omega, 2) * s1);
        double ay = 2.0 * B * (alpha * c2 - 2.0 * std::pow(omega, 2) * s2);

        msg.position = {(float)px, (float)py, (float)pz};
        msg.velocity = {(float)vx, (float)vy, (float)vz};
        msg.acceleration = {(float)ax, (float)ay, (float)az};

        // ================= 偏航角始终锁定为起飞朝向 =================
        msg.yaw = (float)start_yaw_; 
        msg.yawspeed = 0.0;          

        // ================= 发送数据 =================
        auto timestamp = this->get_clock()->now();
        msg.timestamp = timestamp.nanoseconds() / 1000;
        setpoint_pub_->publish(msg);

        expected_path_.header.stamp = timestamp;
        path_pub_->publish(expected_path_);

// 🌟 新增：打印实时期望轨迹（每 500 毫秒打印一次，防止 100Hz 刷爆 Ubuntu 终端）
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), 
            *this->get_clock(), 
            500, 
            "🎯 Setpoint -> X: % .3f, Y: % .3f, Z: % .3f | Mode Time: % .2f s", 
            px, py, pz, t
        );

        time_step_++;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPlanner>());
    rclcpp::shutdown();
    return 0;
}