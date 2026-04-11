#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
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
        // --- Publishers ---
        setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
            "/geometric_controller/setpoint", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/geometric_controller/expected_path", 10);

        // --- Subscribers ---
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            [this](const mavros_msgs::msg::State::SharedPtr msg) {
                if (msg->mode == "OFFBOARD" && msg->armed && !trajectory_started_) {
                    if (has_pose_) {
                        // 切入 Offboard 瞬间锁定物理坐标作为起点
                        start_x_ = current_pose_.pose.position.x;
                        start_y_ = current_pose_.pose.position.y;
                        start_z_ = current_pose_.pose.position.z;
                        start_yaw_ = current_yaw_;

                        RCLCPP_INFO(this->get_logger(), "📍 Initial Pose Locked! X:%.2f, Y:%.2f, Yaw:%.2f", start_x_, start_y_, start_yaw_);
                        RCLCPP_INFO(this->get_logger(), "🚀 Offboard Mode detected! Starting 5-second smooth takeoff...");
                        trajectory_started_ = true;
                        
                        // 锁定起点后再生成并发布 RViz 视觉轨迹
                        init_visual_path();
                    }
                }
            });

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", rclcpp::QoS(10).best_effort(),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                current_pose_ = *msg;
                has_pose_ = true;
                // 四元数转 Yaw
                double q_w = msg->pose.orientation.w; double q_x = msg->pose.orientation.x;
                double q_y = msg->pose.orientation.y; double q_z = msg->pose.orientation.z;
                current_yaw_ = std::atan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z));
            });

        time_step_ = 0;

        timer_ = this->create_wall_timer(
            10ms, std::bind(&TrajectoryPlanner::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Clean MAVROS Figure-8 Planner (ENU) with Error Logging Started.");
        RCLCPP_INFO(this->get_logger(), "Waiting for ARM and OFFBOARD mode from QGroundControl...");
    }

private:
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; 
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_; 
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    bool has_pose_ = false;
    geometry_msgs::msg::PoseStamped current_pose_;
    
    double current_yaw_ = 0.0;
    double start_x_ = 0.0, start_y_ = 0.0, start_z_ = 0.0, start_yaw_ = 0.0;
        
    nav_msgs::msg::Path expected_path_; 
    uint64_t time_step_;

    bool trajectory_started_ = false;

    // 🌟 新增：用于统计巡航阶段误差的变量
    double sum_error_x_ = 0.0;
    double sum_error_y_ = 0.0;
    double sum_error_z_ = 0.0;
    uint64_t cruise_sample_count_ = 0;

    void init_visual_path()
    {
        double A = 2.0;  
        double B = 1.0;  
        double omega_max = 0.25; 
        double target_z = 1.8; // 与飞行高度一致
        
        expected_path_.header.frame_id = "map"; 
        expected_path_.poses.clear();
        
        double T = 2.0 * M_PI / omega_max; 
        for (double t = 0; t <= T; t += 0.05) {
            geometry_msgs::msg::PoseStamped pose;
            // 叠加上物理起点的偏移量，让 RViz 的轨迹圈和真实飞行位置完全重合
            pose.pose.position.x = start_x_ + A * std::sin(omega_max * t);
            pose.pose.position.y = start_y_ + B * std::sin(2.0 * omega_max * t);
            pose.pose.position.z = start_z_ + target_z; 
            expected_path_.poses.push_back(pose);
        }
    }

    void timer_callback()
    {
        mavros_msgs::msg::PositionTarget msg{};

        // ================= 时间冻结与相对位置预热逻辑 =================
        if (!trajectory_started_) {
            time_step_ = 0; 
            
            if (has_pose_) {
                msg.position.x = current_pose_.pose.position.x;
                msg.position.y = current_pose_.pose.position.y;
                msg.position.z = current_pose_.pose.position.z;
                msg.yaw = current_yaw_;
            } else {
                msg.position.x = 0; msg.position.y = 0; msg.position.z = 0;
                msg.yaw = 0;
            }
            
            msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED; 
            msg.type_mask = 0; 
            msg.velocity.x = 0; msg.velocity.y = 0; msg.velocity.z = 0;
            msg.acceleration_or_force.x = 0; msg.acceleration_or_force.y = 0; msg.acceleration_or_force.z = 0;
            msg.yaw_rate = 0;
            
            auto timestamp = this->get_clock()->now();
            msg.header.stamp = timestamp;
            msg.header.frame_id = "map";
            
            setpoint_pub_->publish(msg);

            static int wait_print_counter = 0;
            if (wait_print_counter++ % 200 == 0) {
                RCLCPP_INFO(this->get_logger(), "Waiting for Arm & Offboard... Pre-heating controller.");
            }
            return; 
        }
        
        double t = static_cast<double>(time_step_) * 0.01; 
        
        // ================= 核心物理参数 (8字轨迹) =================
        const double A = 2.0;        
        const double B = 1.0;        
        const double omega_max = 0.25; 
        const double target_z = 1.8;  // MAVROS ENU坐标系，高度为正数
        
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
        
        // 🌟 新增：任务结束时，计算并打印统计误差
        if (t > T_total + T_wait_after_land) {
            if (cruise_sample_count_ > 0) {
                double mae_x = sum_error_x_ / cruise_sample_count_;
                double mae_y = sum_error_y_ / cruise_sample_count_;
                double mae_z = sum_error_z_ / cruise_sample_count_;
                
                RCLCPP_INFO(this->get_logger(), "==================================================");
                RCLCPP_INFO(this->get_logger(), "📊 Cruise Phase Tracking Error Summary (MAE):");
                RCLCPP_INFO(this->get_logger(), "   Mean Absolute Error X: %.4f m", mae_x);
                RCLCPP_INFO(this->get_logger(), "   Mean Absolute Error Y: %.4f m", mae_y);
                RCLCPP_INFO(this->get_logger(), "   Mean Absolute Error Z: %.4f m", mae_z);
                RCLCPP_INFO(this->get_logger(), "   Total Samples Evaluated: %lu", cruise_sample_count_);
                RCLCPP_INFO(this->get_logger(), "==================================================");
            }
            RCLCPP_INFO(this->get_logger(), "Mission Accomplished. Shutting down Planner...");
            rclcpp::shutdown(); 
            return; 
        }

        // 精简的低阶运动学变量
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
        
        // MAVROS PositionTarget 封装
        msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED; 
        msg.type_mask = 0; // 使用位置、速度、加速度、Yaw
        msg.position.x = px; msg.position.y = py; msg.position.z = pz;
        msg.velocity.x = vx; msg.velocity.y = vy; msg.velocity.z = vz;
        msg.acceleration_or_force.x = ax; msg.acceleration_or_force.y = ay; msg.acceleration_or_force.z = az;

        // ================= 偏航角始终锁定为起飞朝向 =================
        msg.yaw = start_yaw_; 
        msg.yaw_rate = 0.0; 

        // ================= 🌟 实时位置与误差计算打印 =================
        double real_x = current_pose_.pose.position.x;
        double real_y = current_pose_.pose.position.y;
        double real_z = current_pose_.pose.position.z;

        double err_x = std::abs(px - real_x);
        double err_y = std::abs(py - real_y);
        double err_z = std::abs(pz - real_z);

        // 如果处于巡航阶段，累加误差用于最终计算 MAE
        if (t >= (T_takeoff + T_accel) && t < (T_takeoff + T_accel + T_cruise)) {
            sum_error_x_ += err_x;
            sum_error_y_ += err_y;
            sum_error_z_ += err_z;
            cruise_sample_count_++;
        }

        // 每 500 毫秒打印一次：时间 | 期望位置 | 真实位置 | 绝对误差
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), 
            *this->get_clock(), 
            500, 
            "⏱️ t:%5.1fs | SP[% .2f, % .2f, % .2f] | Real[% .2f, % .2f, % .2f] | Err[%.3f, %.3f, %.3f]", 
            t, px, py, pz, real_x, real_y, real_z, err_x, err_y, err_z
        );

        auto timestamp = this->get_clock()->now();
        msg.header.stamp = timestamp;
        msg.header.frame_id = "map";
        
        setpoint_pub_->publish(msg);
        
        // 发布预期轨迹给 RViz (仅在起飞后)
        expected_path_.header.stamp = timestamp;
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