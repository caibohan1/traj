#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/path.hpp>                 
#include <geometry_msgs/msg/pose_stamped.hpp>    
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class CircleTrajectoryPlanner : public rclcpp::Node
{
public:
    CircleTrajectoryPlanner() : Node("circle_trajectory_planner")
    {
        setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
            "/geometric_controller/setpoint", 10);

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/geometric_controller/expected_path", 10);

        // ================= 监听飞控状态 =================
        status_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", rclcpp::QoS(10).best_effort(),
            [this](const mavros_msgs::msg::State::SharedPtr msg) {
                bool is_armed = msg->armed;
                bool is_offboard = (msg->mode == "OFFBOARD");
                
                if (is_armed && is_offboard && !trajectory_started_) {
                    if (has_pose_) {
                        start_x_ = current_pose_.pose.position.x;
                        start_y_ = current_pose_.pose.position.y;
                        start_z_ = current_pose_.pose.position.z;
                        
                        double q_w = current_pose_.pose.orientation.w;
                        double q_x = current_pose_.pose.orientation.x;
                        double q_y = current_pose_.pose.orientation.y;
                        double q_z = current_pose_.pose.orientation.z;
                        start_yaw_ = std::atan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z));
                        
                        RCLCPP_INFO(this->get_logger(), "🚀 Circle Trajectory Started! Origin Locked: X:%.2f, Y:%.2f, Yaw:%.2f", start_x_, start_y_, start_yaw_);
                        init_visual_path(); 
                        trajectory_started_ = true;
                    }
                }
            });

        // ================= 监听真实里程计 =================
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", rclcpp::QoS(10).best_effort(),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                current_pose_ = *msg;
                has_pose_ = true;
            });

        time_step_ = 0;
        current_theta_ = 0.0;

        timer_ = this->create_wall_timer(
            10ms, std::bind(&CircleTrajectoryPlanner::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Geometric Circle Planner (MAVROS ENU Edition) Started.");
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
    double current_theta_ = 0.0;

    // 🌟 MAE 误差统计
    double sum_error_x_ = 0.0, sum_error_y_ = 0.0, sum_error_z_ = 0.0;
    uint64_t cruise_sample_count_ = 0;

    // 核心轨迹参数
    const double R = 0.5;                
    const double omega_max = 1.0;        
    const double target_z = 1.8;         // 🌟 MAVROS (ENU) 向上为正 1.8m
    const double circles_accel = 1.0;    
    const double circles_cruise = 3.0;   
    const double circles_decel = 1.0;    

    void init_visual_path()
    {
        expected_path_.header.frame_id = "map"; 
        expected_path_.poses.clear();

        double total_circles = circles_accel + circles_cruise + circles_decel;
        for (double th = 0; th <= 2.0 * M_PI * total_circles; th += 0.05) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = start_x_ + R * std::sin(th);
            pose.pose.position.y = start_y_ + R - R * std::cos(th);
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
        msg.type_mask = 0; 

        // ================= 未起飞前，指令冻结 =================
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
                msg.position.x = 0; msg.position.y = 0; msg.position.z = 0;
                msg.yaw = 0;
            }
            msg.velocity.x = 0; msg.velocity.y = 0; msg.velocity.z = 0;
            msg.acceleration_or_force.x = 0; msg.acceleration_or_force.y = 0; msg.acceleration_or_force.z = 0;
            msg.yaw_rate = 0;
            
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
            if (cruise_sample_count_ > 0) {
                double mae_x = sum_error_x_ / cruise_sample_count_;
                double mae_y = sum_error_y_ / cruise_sample_count_;
                double mae_z = sum_error_z_ / cruise_sample_count_;
                RCLCPP_INFO(this->get_logger(), "==================================================");
                RCLCPP_INFO(this->get_logger(), "📊 MAVROS 几何控制 - 【盘旋阶段】 MAE评估:");
                RCLCPP_INFO(this->get_logger(), "   X: %.4f m | Y: %.4f m | Z: %.4f m", mae_x, mae_y, mae_z);
                RCLCPP_INFO(this->get_logger(), "==================================================");
            }
            RCLCPP_INFO(this->get_logger(), "🏁 Planner Finished. Disarming managed by Controller Failsafe.");
            rclcpp::shutdown(); 
            return; 
        }

        double pz=0, vz=0, az=0, jz=0;
        double omega=0, alpha=0, gamma=0;

        // ================= 五段式柔性状态机 =================
        if (t < T_takeoff) {
            double tau = t / T_takeoff;
            double tau2 = tau*tau, tau3 = tau2*tau, tau4 = tau3*tau, tau5 = tau4*tau;
            double S = 10.0*tau3 - 15.0*tau4 + 6.0*tau5;
            double dS = (1.0/T_takeoff) * (30.0*tau2 - 60.0*tau3 + 30.0*tau4);
            double ddS = (1.0/std::pow(T_takeoff, 2)) * (60.0*tau - 180.0*tau2 + 120.0*tau3);
            double dddS = (1.0/std::pow(T_takeoff, 3)) * (60.0 - 360.0*tau + 360.0*tau2);
            pz = target_z * S; vz = target_z * dS; az = target_z * ddS; jz = target_z * dddS;
        } 
        else if (t < T_takeoff + T_accel) {
            double tau = (t - T_takeoff) / T_accel;
            double tau2 = tau*tau, tau3 = tau2*tau, tau4 = tau3*tau, tau5 = tau4*tau;
            double S = 10.0*tau3 - 15.0*tau4 + 6.0*tau5;
            double dS = (1.0/T_accel) * (30.0*tau2 - 60.0*tau3 + 30.0*tau4);
            double ddS = (1.0/std::pow(T_accel, 2)) * (60.0*tau - 180.0*tau2 + 120.0*tau3);
            pz = target_z; vz = 0; az = 0; jz = 0;
            omega = omega_max * S; alpha = omega_max * dS; gamma = omega_max * ddS;
        } 
        else if (t < T_takeoff + T_accel + T_cruise) {
            pz = target_z; vz = 0; az = 0; jz = 0;
            omega = omega_max; alpha = 0; gamma = 0;
        } 
        else if (t < T_takeoff + T_accel + T_cruise + T_decel) {
            double tau = (t - T_takeoff - T_accel - T_cruise) / T_decel;
            double tau2 = tau*tau, tau3 = tau2*tau, tau4 = tau3*tau, tau5 = tau4*tau;
            double S = 10.0*tau3 - 15.0*tau4 + 6.0*tau5;
            double dS = (1.0/T_decel) * (30.0*tau2 - 60.0*tau3 + 30.0*tau4);
            double ddS = (1.0/std::pow(T_decel, 2)) * (60.0*tau - 180.0*tau2 + 120.0*tau3);
            pz = target_z; vz = 0; az = 0; jz = 0;
            omega = omega_max * (1.0 - S); alpha = -omega_max * dS; gamma = -omega_max * ddS;
        } 
        else if (t < T_total) {
            double tau = (t - T_takeoff - T_accel - T_cruise - T_decel) / T_land;
            double tau2 = tau*tau, tau3 = tau2*tau, tau4 = tau3*tau, tau5 = tau4*tau;
            double S = 10.0*tau3 - 15.0*tau4 + 6.0*tau5;
            double dS = (1.0/T_land) * (30.0*tau2 - 60.0*tau3 + 30.0*tau4);
            double ddS = (1.0/std::pow(T_land, 2)) * (60.0*tau - 180.0*tau2 + 120.0*tau3);
            double dddS = (1.0/std::pow(T_land, 3)) * (60.0 - 360.0*tau + 360.0*tau2);
            pz = target_z * (1.0 - S); vz = -target_z * dS; az = -target_z * ddS; jz = -target_z * dddS;
            omega = 0; alpha = 0; gamma = 0;
        }

        // ================= 圆形轨迹运动学映射 =================
        current_theta_ += omega * 0.01;
        double th = current_theta_;
        double c1 = std::cos(th), s1 = std::sin(th);

        double px = R * s1;
        double py = R - R * c1;

        double vx = R * c1 * omega;
        double vy = R * s1 * omega;

        double ax = R * (alpha * c1 - std::pow(omega, 2) * s1);
        double ay = R * (alpha * s1 + std::pow(omega, 2) * c1);

        double jx = R * (gamma * c1 - 3.0 * alpha * omega * s1 - std::pow(omega, 3) * c1);
        double jy = R * (gamma * s1 + 3.0 * alpha * omega * c1 - std::pow(omega, 3) * s1);
        (void)jx; (void)jy; (void)jz; // 消除警告，MAVROS 消息无 Jerk 字段

        // 最终期望位置
        double target_px = start_x_ + px;
        double target_py = start_y_ + py;
        double target_pz = start_z_ + pz;

        // 🌟 实时误差计算与累加
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

        // 实时终端打印对比
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 500, 
            "⏱️ t: %4.1f s | SP[% .2f, % .2f, % .2f] | Real[% .2f, % .2f, % .2f] | Err[%.3f, %.3f, %.3f]", 
            t, target_px, target_py, target_pz, real_x, real_y, real_z, err_x, err_y, err_z
        );

        // 装填并发送消息
        msg.position.x = target_px;
        msg.position.y = target_py;
        msg.position.z = target_pz;

        msg.velocity.x = vx;
        msg.velocity.y = vy;
        msg.velocity.z = vz;

        msg.acceleration_or_force.x = ax;
        msg.acceleration_or_force.y = ay;
        msg.acceleration_or_force.z = az;

        msg.yaw = (float)start_yaw_;
        msg.yaw_rate = 0.0f;

        setpoint_pub_->publish(msg);

        expected_path_.header.stamp = this->get_clock()->now();
        path_pub_->publish(expected_path_);

        time_step_++;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CircleTrajectoryPlanner>());
    rclcpp::shutdown();
    return 0;
}