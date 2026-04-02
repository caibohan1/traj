#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <nav_msgs/msg/path.hpp>                 
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class TrajectoryPlanner : public rclcpp::Node
{
public:
    TrajectoryPlanner() : Node("trajectory_planner")
    {
        // 替换为 MAVROS 的 PositionTarget
        setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
            "/geometric_controller/setpoint", 10);

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/geometric_controller/expected_path", 10);

        ext_setpoint_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/geometric_controller/traj_ext", 10);

        time_step_ = 0;
        init_visual_path();

        timer_ = this->create_wall_timer(
            10ms, std::bind(&TrajectoryPlanner::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Symmetric 2-2-2 Figure-8 Planner (MAVROS/ENU) Started.");
    }

private:
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; 
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ext_setpoint_pub_; 
    rclcpp::TimerBase::SharedPtr timer_;
    
    nav_msgs::msg::Path expected_path_; 
    uint64_t time_step_;

    double last_yaw_ = 0.0;
    bool first_yaw_ = true;

    void init_visual_path()
    {
        double A = 2.0;  
        double B = 1.0;  
        double omega_max = 0.5; 
        expected_path_.header.frame_id = "map"; 
        
        // 绘制完整 8 字形参考轨迹 (RViz)
        double T = 2.0 * M_PI / omega_max; 
        for (double t = 0; t <= T; t += 0.1) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = A * std::sin(omega_max * t);
            pose.pose.position.y = B * std::sin(2.0 * omega_max * t);
            // ENU 坐标系：天上为正
            pose.pose.position.z = 5.0; 
            expected_path_.poses.push_back(pose);
        }
    }

    void timer_callback()
    {
        mavros_msgs::msg::PositionTarget msg{};
        std_msgs::msg::Float64MultiArray ext_msg{}; 
        ext_msg.data.resize(4); 
        
        double t = static_cast<double>(time_step_) * 0.01; 
        
        // ================= 核心对称物理参数 =================
        const double A = 2.0;        
        const double B = 1.0;        
        const double omega_max = 0.5;   // 最大基础角速度
        const double target_z = 5.0;    // ENU 目标高度     
        
        const double circles_takeoff = 2.0;
        const double circles_cruise  = 2.0;
        const double circles_land    = 2.0;

        // 起降时长需乘以 2 (积分特性补偿)
        const double T_takeoff = (circles_takeoff * 2.0 * M_PI * 2.0) / omega_max; 
        const double T_cruise  = (circles_cruise * 2.0 * M_PI) / omega_max;       
        const double T_land    = (circles_land * 2.0 * M_PI * 2.0) / omega_max;    
        
        // ================= 任务结束与自动关闭逻辑 =================
        const double T_total = T_takeoff + T_cruise + T_land;
        const double T_wait_after_land = 3.0; // 落地后等待 3 秒再关闭
        
        if (t > T_total + T_wait_after_land) {
            RCLCPP_INFO(this->get_logger(), "Mission Accomplished. Shutting down Trajectory Planner...");
            rclcpp::shutdown(); 
            return; 
        }

        double px=0, py=0, pz=0;
        double vx=0, vy=0, vz=0;
        double ax=0, ay=0, az=0;
        double jx=0, jy=0, jz=0;
        double sx=0, sy=0, sz=0;

        double theta = 0, omega = 0, alpha = 0, j_theta = 0, s_theta = 0;

        // ================= 对称状态机推导 =================
        if (t < T_takeoff) {
            // ------ 【阶段 1：2圈加速螺旋起飞】 ------
            double tau = t / T_takeoff;
            double tau2 = tau * tau; double tau3 = tau2 * tau;
            double tau4 = tau3 * tau; double tau5 = tau4 * tau;
            double tau6 = tau5 * tau;

            double S = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
            double dS = (1.0 / T_takeoff) * (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4);
            double ddS = (1.0 / std::pow(T_takeoff, 2)) * (60.0 * tau - 180.0 * tau2 + 120.0 * tau3);
            double dddS = (1.0 / std::pow(T_takeoff, 3)) * (60.0 - 360.0 * tau + 360.0 * tau2);
            double ddddS = (1.0 / std::pow(T_takeoff, 4)) * (-360.0 + 720.0 * tau);

            pz = target_z * S;
            vz = target_z * dS;
            az = target_z * ddS;
            jz = target_z * dddS;
            sz = target_z * ddddS;

            omega = omega_max * S;
            alpha = omega_max * dS;
            j_theta = omega_max * ddS;
            s_theta = omega_max * dddS;

            double I_S = 2.5 * tau4 - 3.0 * tau5 + tau6;
            theta = omega_max * T_takeoff * I_S; 
        } 
        else if (t < T_takeoff + T_cruise) {
            // ------ 【阶段 2：2圈匀速 8 字巡航】 ------
            double t_c = t - T_takeoff;
            theta = (circles_takeoff * 2.0 * M_PI) + (omega_max * t_c);
            
            omega = omega_max;
            alpha = 0.0; j_theta = 0.0; s_theta = 0.0;
            
            pz = target_z; vz = 0.0; az = 0.0; jz = 0.0; sz = 0.0;
        } 
        else if (t < T_takeoff + T_cruise + T_land) {
            // ------ 【阶段 3：2圈减速螺旋降落】 ------
            double t_l = t - T_takeoff - T_cruise;
            double tau = t_l / T_land;
            
            double tau2 = tau * tau; double tau3 = tau2 * tau;
            double tau4 = tau3 * tau; double tau5 = tau4 * tau;
            double tau6 = tau5 * tau;

            double S = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
            double dS = (1.0 / T_land) * (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4);
            double ddS = (1.0 / std::pow(T_land, 2)) * (60.0 * tau - 180.0 * tau2 + 120.0 * tau3);
            double dddS = (1.0 / std::pow(T_land, 3)) * (60.0 - 360.0 * tau + 360.0 * tau2);
            double ddddS = (1.0 / std::pow(T_land, 4)) * (-360.0 + 720.0 * tau);

            pz = target_z * (1.0 - S);
            vz = -target_z * dS;
            az = -target_z * ddS;
            jz = -target_z * dddS;
            sz = -target_z * ddddS;

            omega = omega_max * (1.0 - S);
            alpha = -omega_max * dS;
            j_theta = -omega_max * ddS;
            s_theta = -omega_max * dddS;

            double I_1_minus_S = tau - (2.5 * tau4 - 3.0 * tau5 + tau6);
            theta = (circles_takeoff + circles_cruise) * 2.0 * M_PI + (omega_max * T_land * I_1_minus_S); 
        } 
        else {
            // ------ 【阶段 4：精准锁定原点】 ------
            theta = (circles_takeoff + circles_cruise + circles_land) * 2.0 * M_PI; 
            omega = 0.0; alpha = 0.0; j_theta = 0.0; s_theta = 0.0;
            pz = 0.0; vz = 0.0; az = 0.0; jz = 0.0; sz = 0.0;
        }

        // ================= 严格解析：广义 8 字形参数求导 =================
        double c1 = std::cos(theta);
        double s1 = std::sin(theta);
        double c2 = std::cos(2.0 * theta);
        double s2 = std::sin(2.0 * theta);

        // 位置
        px = A * s1;
        py = B * s2;
        
        // 速度
        vx = A * omega * c1;
        vy = 2.0 * B * omega * c2;
        
        // 加速度
        ax = A * (alpha * c1 - std::pow(omega, 2) * s1);
        ay = 2.0 * B * (alpha * c2 - 2.0 * std::pow(omega, 2) * s2);
        
        // Jerk (加加速度 - 仅用于下游求导计算)
        jx = A * ((j_theta - std::pow(omega, 3)) * c1 - 3.0 * alpha * omega * s1);
        jy = 2.0 * B * ((j_theta - 4.0 * std::pow(omega, 3)) * c2 - 6.0 * alpha * omega * s2);
        
        // Snap (推力变化率前馈核心)
        sx = A * ((s_theta - 6.0 * alpha * std::pow(omega, 2)) * c1 - (4.0 * omega * j_theta + 3.0 * std::pow(alpha, 2) - std::pow(omega, 4)) * s1);
        sy = 2.0 * B * ((s_theta - 24.0 * alpha * std::pow(omega, 2)) * c2 - (8.0 * omega * j_theta + 6.0 * std::pow(alpha, 2) - 16.0 * std::pow(omega, 4)) * s2);

        // ================= 封装 MAVROS 标准消息 =================
        auto timestamp = this->get_clock()->now();
        msg.header.stamp = timestamp;
        msg.header.frame_id = "map";
        msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED; // MAVROS 标准枚举
        
        msg.position.x = px; msg.position.y = py; msg.position.z = pz;
        msg.velocity.x = vx; msg.velocity.y = vy; msg.velocity.z = vz;
        msg.acceleration_or_force.x = ax; msg.acceleration_or_force.y = ay; msg.acceleration_or_force.z = az;
        // 注意：MAVROS PositionTarget 没有 jerk 字段，此处已被移除

        // ================= 偏航角解卷绕与切线追踪 =================
        double raw_yaw = 0.0; 
        double speed_sq = vx * vx + vy * vy;
        
        if (speed_sq > 1e-6) {
            raw_yaw = std::atan2(vy, vx);
        } else {
            raw_yaw = first_yaw_ ? 0.0 : std::atan2(std::sin(last_yaw_), std::cos(last_yaw_));
        }
        
        if (first_yaw_) {
            last_yaw_ = raw_yaw;
            first_yaw_ = false;
        } else {
            double dyaw = raw_yaw - last_yaw_;
            while (dyaw > M_PI)  dyaw -= 2.0 * M_PI;
            while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
            raw_yaw = last_yaw_ + dyaw;
            last_yaw_ = raw_yaw;
        }
        msg.yaw = raw_yaw;
        
        // ================= 偏航角加速度 (曲率求导) =================
        double D_val = speed_sq;
        double N_val = vx * ay - vy * ax;
        
        double yawspeed = 0.0;
        double yawaccel = 0.0; 
        
        if (D_val > 1e-6) {
            yawspeed = N_val / D_val;
            double N_dot = vx * jy - vy * jx; 
            double D_dot = 2.0 * vx * ax + 2.0 * vy * ay;
            yawaccel = (N_dot * D_val - N_val * D_dot) / (D_val * D_val);
        }
        msg.yaw_rate = yawspeed; // 映射到 MAVROS 的 yaw_rate 字段

        // ================= 发送数据 =================
        setpoint_pub_->publish(msg);

        // 扩展通道（Snap 和 YawAccel）完美适配你的 Geometric Controller
        ext_msg.data[0] = sx;
        ext_msg.data[1] = sy;
        ext_msg.data[2] = sz;
        ext_msg.data[3] = yawaccel;
        ext_setpoint_pub_->publish(ext_msg);

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