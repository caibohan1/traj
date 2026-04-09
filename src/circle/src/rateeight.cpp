#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <nav_msgs/msg/path.hpp>                 
#include <geometry_msgs/msg/pose_stamped.hpp>    
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class TrajectoryPlanner : public rclcpp::Node
{
public:
    TrajectoryPlanner() : Node("trajectory_planner")
    {
        // 发布期望轨迹给几何控制器
        setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/geometric_controller/setpoint", 10);

        // 发布RViz可视化路径
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/geometric_controller/expected_path", 10);

        // ================= 监听飞控状态 =================
        status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", rclcpp::QoS(10).best_effort(),
            [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
                bool is_armed = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
                bool is_offboard = (msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
                
                // 只有当几何控制器完成了 Arm 和 Offboard，规划器才开始流转时间
                if (is_armed && is_offboard && !trajectory_started_) {
                    if (has_odom_) {
                        // 锁定起飞瞬间的物理原点
                        start_x_ = current_odom_.position[0];
                        start_y_ = current_odom_.position[1];
                        start_z_ = current_odom_.position[2];
                        double q_w = current_odom_.q[0], q_x = current_odom_.q[1], q_y = current_odom_.q[2], q_z = current_odom_.q[3];
                        start_yaw_ = std::atan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z));
                        
                        RCLCPP_INFO(this->get_logger(), "🚀 Trajectory Started! Origin Locked: X:%.2f, Y:%.2f, Yaw:%.2f", start_x_, start_y_, start_yaw_);
                        init_visual_path(); // 锁定原点后生成 RViz 参考路径
                        trajectory_started_ = true;
                    }
                }
            });

        // ================= 监听真实里程计 =================
        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(),
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                current_odom_ = *msg;
                has_odom_ = true;
            });

        time_step_ = 0;
        current_theta_ = 0.0;

        // 100Hz 频率发布轨迹
        timer_ = this->create_wall_timer(
            10ms, std::bind(&TrajectoryPlanner::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Hardcore Figure-8 Planner (Fixed Yaw Edition) Started. Waiting for Geometric Controller to Arm...");
    }

private:
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; 
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Path expected_path_; 
    px4_msgs::msg::VehicleOdometry current_odom_;
    
    bool has_odom_ = false;
    bool trajectory_started_ = false;

    double start_x_ = 0.0, start_y_ = 0.0, start_z_ = 0.0, start_yaw_ = 0.0;
    uint64_t time_step_;
    
    // 全局相角积分器
    double current_theta_ = 0.0;

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

    void timer_callback()
    {
        px4_msgs::msg::TrajectorySetpoint msg{};

        // ================= 未起飞前，指令冻结在真实原点 =================
        if (!trajectory_started_) {
            time_step_ = 0;
            current_theta_ = 0.0;
            if (has_odom_) {
                msg.position = {current_odom_.position[0], current_odom_.position[1], current_odom_.position[2]};
                double q_w = current_odom_.q[0], q_x = current_odom_.q[1], q_y = current_odom_.q[2], q_z = current_odom_.q[3];
                msg.yaw = (float)std::atan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z));
            } else {
                msg.position = {0.0, 0.0, 0.0};
                msg.yaw = 0.0;
            }
            msg.velocity = {0.0, 0.0, 0.0};
            msg.acceleration = {0.0, 0.0, 0.0};
            msg.jerk = {0.0, 0.0, 0.0};
            msg.yawspeed = 0.0;
            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            setpoint_pub_->publish(msg);
            return;
        }

        double t = static_cast<double>(time_step_) * 0.01; 
        
        // --- 核心物理参数 ---
        const double A = 2.0;        
        const double B = 1.0;        
        const double omega_max = 0.25; // 高速大机动角速度
        const double target_z = -1.8;  // 飞行高度
        
        const double T_takeoff = 5.0; 
        const double T_land    = 5.0; 
        const double circles_accel  = 1.0; 
        const double circles_cruise = 3.0; // 巡航3圈
        const double circles_decel  = 1.0; 

        const double T_accel  = (circles_accel * 2.0 * M_PI * 2.0) / omega_max; 
        const double T_cruise = (circles_cruise * 2.0 * M_PI) / omega_max;       
        const double T_decel  = (circles_decel * 2.0 * M_PI * 2.0) / omega_max;    
        
        const double T_total = T_takeoff + T_accel + T_cruise + T_decel + T_land;
        const double T_wait_after_land = 3.0; 
        
        if (t > T_total + T_wait_after_land) {
            RCLCPP_INFO(this->get_logger(), "🏁 Planner Finished. Waiting for Geometric Controller to Disarm...");
            rclcpp::shutdown(); 
            return; 
        }

        double pz=0, vz=0, az=0, jz=0;
        double omega=0, alpha=0, gamma=0;

        // ================= 五段式柔性状态机 =================
        if (t < T_takeoff) {
            // 1. 垂直平滑起飞
            double tau = t / T_takeoff;
            double tau2 = tau*tau, tau3 = tau2*tau, tau4 = tau3*tau, tau5 = tau4*tau;
            
            double S = 10.0*tau3 - 15.0*tau4 + 6.0*tau5;
            double dS = (1.0/T_takeoff) * (30.0*tau2 - 60.0*tau3 + 30.0*tau4);
            double ddS = (1.0/std::pow(T_takeoff, 2)) * (60.0*tau - 180.0*tau2 + 120.0*tau3);
            double dddS = (1.0/std::pow(T_takeoff, 3)) * (60.0 - 360.0*tau + 360.0*tau2);

            pz = target_z * S; vz = target_z * dS; az = target_z * ddS; jz = target_z * dddS;
        } 
        else if (t < T_takeoff + T_accel) {
            // 2. 水平相角柔性加速
            double tau = (t - T_takeoff) / T_accel;
            double tau2 = tau*tau, tau3 = tau2*tau, tau4 = tau3*tau, tau5 = tau4*tau;
            
            double S = 10.0*tau3 - 15.0*tau4 + 6.0*tau5;
            double dS = (1.0/T_accel) * (30.0*tau2 - 60.0*tau3 + 30.0*tau4);
            double ddS = (1.0/std::pow(T_accel, 2)) * (60.0*tau - 180.0*tau2 + 120.0*tau3);

            pz = target_z; vz = 0; az = 0; jz = 0;
            omega = omega_max * S; alpha = omega_max * dS; gamma = omega_max * ddS;
        } 
        else if (t < T_takeoff + T_accel + T_cruise) {
            // 3. 恒速巡航
            pz = target_z; vz = 0; az = 0; jz = 0;
            omega = omega_max; alpha = 0; gamma = 0;
        } 
        else if (t < T_takeoff + T_accel + T_cruise + T_decel) {
            // 4. 水平相角柔性减速
            double tau = (t - T_takeoff - T_accel - T_cruise) / T_decel;
            double tau2 = tau*tau, tau3 = tau2*tau, tau4 = tau3*tau, tau5 = tau4*tau;
            
            double S = 10.0*tau3 - 15.0*tau4 + 6.0*tau5;
            double dS = (1.0/T_decel) * (30.0*tau2 - 60.0*tau3 + 30.0*tau4);
            double ddS = (1.0/std::pow(T_decel, 2)) * (60.0*tau - 180.0*tau2 + 120.0*tau3);

            pz = target_z; vz = 0; az = 0; jz = 0;
            omega = omega_max * (1.0 - S); alpha = -omega_max * dS; gamma = -omega_max * ddS;
        } 
        else if (t < T_total) {
            // 5. 垂直平滑降落
            double tau = (t - T_takeoff - T_accel - T_cruise - T_decel) / T_land;
            double tau2 = tau*tau, tau3 = tau2*tau, tau4 = tau3*tau, tau5 = tau4*tau;
            
            double S = 10.0*tau3 - 15.0*tau4 + 6.0*tau5;
            double dS = (1.0/T_land) * (30.0*tau2 - 60.0*tau3 + 30.0*tau4);
            double ddS = (1.0/std::pow(T_land, 2)) * (60.0*tau - 180.0*tau2 + 120.0*tau3);
            double dddS = (1.0/std::pow(T_land, 3)) * (60.0 - 360.0*tau + 360.0*tau2);

            pz = target_z * (1.0 - S); vz = -target_z * dS; az = -target_z * ddS; jz = -target_z * dddS;
            omega = 0; alpha = 0; gamma = 0;
        }

        // ================= 精确的 8 字形参数映射 (至三阶导数 Jerk) =================
        current_theta_ += omega * 0.01;
        double th = current_theta_;
        double c1 = std::cos(th), s1 = std::sin(th);
        double c2 = std::cos(2.0 * th), s2 = std::sin(2.0 * th);

        // 1. 位置
        double px = A * s1;
        double py = B * s2;

        // 2. 速度 (一阶导)
        double vx = A * c1 * omega;
        double vy = 2.0 * B * c2 * omega;

        // 3. 加速度 (二阶导)
        double ax = A * (alpha * c1 - std::pow(omega, 2) * s1);
        double ay = 2.0 * B * (alpha * c2 - 2.0 * std::pow(omega, 2) * s2);

        // 4. 加加速度 Jerk (三阶导) - 🌟 几何控制器最依赖的参数！
        double jx = A * (gamma * c1 - 3.0 * alpha * omega * s1 - std::pow(omega, 3) * c1);
        double jy = 2.0 * B * (gamma * c2 - 6.0 * alpha * omega * s2 - 4.0 * std::pow(omega, 3) * c2);

        // 叠加原点偏移量
        msg.position = {(float)(start_x_ + px), (float)(start_y_ + py), (float)(start_z_ + pz)};
        msg.velocity = {(float)vx, (float)vy, (float)vz};
        msg.acceleration = {(float)ax, (float)ay, (float)az};
        msg.jerk = {(float)jx, (float)jy, (float)jz};

        // 🌟 ================= 偏航角始终锁定为起飞朝向 ================= 🌟
        msg.yaw = (float)start_yaw_;
        msg.yawspeed = 0.0f;

        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        setpoint_pub_->publish(msg);

        expected_path_.header.stamp = this->get_clock()->now();
        path_pub_->publish(expected_path_);

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
