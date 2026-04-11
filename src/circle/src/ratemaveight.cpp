#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/path.hpp>                 
#include <geometry_msgs/msg/pose_stamped.hpp>    
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class MavrosFigure8Planner : public rclcpp::Node
{
public:
    MavrosFigure8Planner() : Node("mavros_figure8_planner")
    {
        // 发布期望轨迹给几何控制器 (MAVROS 格式)
        setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
            "/geometric_controller/setpoint", 10);

        // 发布RViz可视化路径
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/geometric_controller/expected_path", 10);

        // ================= 监听飞控状态 (MAVROS) =================
        status_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", rclcpp::QoS(10).best_effort(),
            [this](const mavros_msgs::msg::State::SharedPtr msg) {
                bool is_armed = msg->armed;
                bool is_offboard = (msg->mode == "OFFBOARD");
                
                // 只有当几何控制器完成了 Arm 和 Offboard，规划器才开始流转时间
                if (is_armed && is_offboard && !trajectory_started_) {
                    if (has_pose_) {
                        // 锁定起飞瞬间的物理原点
                        start_x_ = current_pose_.pose.position.x;
                        start_y_ = current_pose_.pose.position.y;
                        start_z_ = current_pose_.pose.position.z;
                        
                        double q_w = current_pose_.pose.orientation.w;
                        double q_x = current_pose_.pose.orientation.x;
                        double q_y = current_pose_.pose.orientation.y;
                        double q_z = current_pose_.pose.orientation.z;
                        start_yaw_ = std::atan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z));
                        
                        RCLCPP_INFO(this->get_logger(), "🚀 Trajectory Started! Origin Locked: X:%.2f, Y:%.2f, Yaw:%.2f", start_x_, start_y_, start_yaw_);
                        init_visual_path(); // 锁定原点后生成 RViz 参考路径
                        trajectory_started_ = true;
                    }
                }
            });

        // ================= 监听真实位姿 (MAVROS) =================
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", rclcpp::QoS(10).best_effort(),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                current_pose_ = *msg;
                has_pose_ = true;
            });

        time_step_ = 0;
        current_theta_ = 0.0;

        // 100Hz 频率发布轨迹
        timer_ = this->create_wall_timer(
            10ms, std::bind(&MavrosFigure8Planner::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Hardcore Figure-8 Planner (MAVROS ENU Edition) Started. Waiting for Controller to Arm...");
    }

private:
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; 
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Path expected_path_; 
    geometry_msgs::msg::PoseStamped current_pose_;
    
    bool has_pose_ = false;
    bool trajectory_started_ = false;

    double start_x_ = 0.0, start_y_ = 0.0, start_z_ = 0.0, start_yaw_ = 0.0;
    uint64_t time_step_;
    
    // 全局相角积分器
    double current_theta_ = 0.0;

    // 🌟 MAE 误差统计
    double sum_error_x_ = 0.0;
    double sum_error_y_ = 0.0;
    double sum_error_z_ = 0.0;
    uint64_t cruise_sample_count_ = 0;

    // --- 核心物理参数 ---
    const double A = 2.0;        
    const double B = 1.0;        
    const double omega_max = 1.8; // 高速大机动角速度
    const double target_z = 1.8;  // 🌟 MAVROS ENU 向上为正
    
    const double circles_accel  = 1.0; 
    const double circles_cruise = 3.0; // 巡航3圈
    const double circles_decel  = 1.0; 

    void init_visual_path()
    {
        expected_path_.header.frame_id = "map"; 
        expected_path_.poses.clear();

        double total_circles = circles_accel + circles_cruise + circles_decel;
        for (double th = 0; th <= 2.0 * M_PI * total_circles; th += 0.05) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = start_x_ + A * std::sin(th);
            pose.pose.position.y = start_y_ + B * std::sin(2.0 * th);
            pose.pose.position.z = start_z_ + target_z; 
            expected_path_.poses.push_back(pose);
        }
    }

    void timer_callback()
    {
        mavros_msgs::msg::PositionTarget msg{};
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";
        msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        msg.type_mask = 0; // 启用 位置+速度+加速度+Yaw

        // ================= 未起飞前，指令冻结在真实原点 =================
        if (!trajectory_started_) {
            time_step_ = 0;
            current_theta_ = 0.0;
            if (has_pose_) {
                msg.position.x = current_pose_.pose.position.x;
                msg.position.y = current_pose_.pose.position.y;
                msg.position.z = current_pose_.pose.position.z;
                
                double q_w = current_pose_.pose.orientation.w;
                double q_x = current_pose_.pose.orientation.x;
                double q_y = current_pose_.pose.orientation.y;
                double q_z = current_pose_.pose.orientation.z;
                msg.yaw = (float)std::atan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z));
            } else {
                msg.position.x = 0.0; msg.position.y = 0.0; msg.position.z = 0.0;
                msg.yaw = 0.0;
            }
            msg.velocity.x = 0.0; msg.velocity.y = 0.0; msg.velocity.z = 0.0;
            msg.acceleration_or_force.x = 0.0; msg.acceleration_or_force.y = 0.0; msg.acceleration_or_force.z = 0.0;
            msg.yaw_rate = 0.0;
            
            setpoint_pub_->publish(msg);
            return;
        }

        double t = static_cast<double>(time_step_) * 0.01; 
        
        const double T_takeoff = 5.0; 
        const double T_land    = 5.0; 

        const double T_accel  = (circles_accel * 2.0 * M_PI * 2.0) / omega_max; 
        const double T_cruise = (circles_cruise * 2.0 * M_PI) / omega_max;       
        const double T_decel  = (circles_decel * 2.0 * M_PI * 2.0) / omega_max;    
        
        const double T_total = T_takeoff + T_accel + T_cruise + T_decel + T_land;
        const double T_wait_after_land = 3.0; 
        
        if (t > T_total + T_wait_after_land) {
            // 🌟 任务结束时计算 MAE
            if (cruise_sample_count_ > 0) {
                double mae_x = sum_error_x_ / cruise_sample_count_;
                double mae_y = sum_error_y_ / cruise_sample_count_;
                double mae_z = sum_error_z_ / cruise_sample_count_;
                RCLCPP_INFO(this->get_logger(), "==================================================");
                RCLCPP_INFO(this->get_logger(), "📊 MAVROS 几何控制 - 【盘旋阶段】 MAE评估:");
                RCLCPP_INFO(this->get_logger(), "   X: %.4f m | Y: %.4f m | Z: %.4f m", mae_x, mae_y, mae_z);
                RCLCPP_INFO(this->get_logger(), "==================================================");
            }
            RCLCPP_INFO(this->get_logger(), "🏁 Planner Finished. Waiting for Geometric Controller Failsafe to Disarm...");
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

        // 4. 加加速度 Jerk (三阶导) - 虽然 MAVROS 不发，但必须计算以消除警告或供后续使用
        double jx = A * (gamma * c1 - 3.0 * alpha * omega * s1 - std::pow(omega, 3) * c1);
        double jy = 2.0 * B * (gamma * c2 - 6.0 * alpha * omega * s2 - 4.0 * std::pow(omega, 3) * c2);
        (void)jx; (void)jy; (void)jz; 

        // 叠加原点偏移量
        double target_px = start_x_ + px;
        double target_py = start_y_ + py;
        double target_pz = start_z_ + pz;

        msg.position.x = target_px;
        msg.position.y = target_py;
        msg.position.z = target_pz;

        msg.velocity.x = vx;
        msg.velocity.y = vy;
        msg.velocity.z = vz;

        msg.acceleration_or_force.x = ax;
        msg.acceleration_or_force.y = ay;
        msg.acceleration_or_force.z = az;

        // 🌟 ================= 偏航角始终锁定为起飞朝向 ================= 🌟
        msg.yaw = (float)start_yaw_;
        msg.yaw_rate = 0.0f;

        setpoint_pub_->publish(msg);

        expected_path_.header.stamp = this->get_clock()->now();
        path_pub_->publish(expected_path_);

        // 🌟 实时误差计算与终端打印
        double real_x = current_pose_.pose.position.x;
        double real_y = current_pose_.pose.position.y;
        double real_z = current_pose_.pose.position.z;

        double err_x = std::abs(target_px - real_x);
        double err_y = std::abs(target_py - real_y);
        double err_z = std::abs(target_pz - real_z);

        if (t >= (T_takeoff + T_accel) && t < (T_takeoff + T_accel + T_cruise)) {
            sum_error_x_ += err_x;
            sum_error_y_ += err_y;
            sum_error_z_ += err_z;
            cruise_sample_count_++;
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 500, 
            "⏱️ t: %4.1f s | SP[% .2f, % .2f, % .2f] | Real[% .2f, % .2f, % .2f] | Err[%.3f, %.3f, %.3f]", 
            t, target_px, target_py, target_pz, real_x, real_y, real_z, err_x, err_y, err_z
        );

        time_step_++;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MavrosFigure8Planner>());
    rclcpp::shutdown();
    return 0;
}