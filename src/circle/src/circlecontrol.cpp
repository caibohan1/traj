#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
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
        setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/geometric_controller/setpoint", 10);

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/geometric_controller/expected_path", 10);

        ext_setpoint_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/geometric_controller/traj_ext", 10);

        time_step_ = 0;
        init_visual_path();

        timer_ = this->create_wall_timer(
            10ms, std::bind(&TrajectoryPlanner::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Symmetric 2-2-2 Spiral Planner (Half Speed) Started.");
    }

private:
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; 
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ext_setpoint_pub_; 
    rclcpp::TimerBase::SharedPtr timer_;
    
    nav_msgs::msg::Path expected_path_; 
    uint64_t time_step_;

    double last_yaw_ = 0.0;
    bool first_yaw_ = true;

    void init_visual_path()
    {
        double R = 3.0;
        double omega_max = 0.625; // 速度减半
        expected_path_.header.frame_id = "map"; 
        
        // RViz 仅绘制地面的参考圆底座
        double T = 2.0 * M_PI / omega_max; 
        for (double t = 0; t <= T; t += 0.1) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = R * std::sin(omega_max * t);
            pose.pose.position.y = R - R * std::cos(omega_max * t); 
            pose.pose.position.z = -5.0; 
            expected_path_.poses.push_back(pose);
        }
    }

    void timer_callback()
    {
        px4_msgs::msg::TrajectorySetpoint msg{};
        std_msgs::msg::Float64MultiArray ext_msg{}; 
        ext_msg.data.resize(4); 
        
        double t = static_cast<double>(time_step_) * 0.01; 
        
        // ================= 核心对称物理参数 =================
        const double R = 3.0;        
        const double omega_max = 0.625; // 速度减小为原来(1.25)的一半
        const double target_z = -5.0;     
        
        // 设定起飞、巡航、降落各 2 圈
        const double circles_takeoff = 2.0;
        const double circles_cruise  = 2.0;
        const double circles_land    = 2.0;

        // 【数学魔法】：基于五阶多项式积分特性，时长需乘以 2 才能完美覆盖指定圈数
        const double T_takeoff = (circles_takeoff * 2.0 * M_PI * 2.0) / omega_max; 
        const double T_cruise  = (circles_cruise * 2.0 * M_PI) / omega_max;       
        const double T_land    = (circles_land * 2.0 * M_PI * 2.0) / omega_max;    
        
// ================= 任务结束与自动关闭逻辑 =================
        const double T_total = T_takeoff + T_cruise + T_land;
        const double T_wait_after_land = 3.0; // 落地后等待 3 秒再关闭
        
        if (t > T_total + T_wait_after_land) {
            RCLCPP_INFO(this->get_logger(), "Mission Accomplished. Shutting down Trajectory Planner...");
            rclcpp::shutdown(); // 优雅关闭 ROS 2 节点
            return; // 立即退出回调函数
        }

        double px=0, py=0, pz=0;
        double vx=0, vy=0, vz=0;
        double ax=0, ay=0, az=0;
        double jx=0, jy=0, jz=0;
        double sx=0, sy=0, sz=0;

        double theta = 0, omega = 0, alpha = 0, j_theta = 0, s_theta = 0;

        // ================= 对称状态机推导 =================
        if (t < T_takeoff) {
            // ------ 【阶段 1：2圈三维螺旋平滑起飞】 ------
            double tau = t / T_takeoff;
            double tau2 = tau * tau; double tau3 = tau2 * tau;
            double tau4 = tau3 * tau; double tau5 = tau4 * tau;
            double tau6 = tau5 * tau;

            double S = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
            double dS = (1.0 / T_takeoff) * (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4);
            double ddS = (1.0 / std::pow(T_takeoff, 2)) * (60.0 * tau - 180.0 * tau2 + 120.0 * tau3);
            double dddS = (1.0 / std::pow(T_takeoff, 3)) * (60.0 - 360.0 * tau + 360.0 * tau2);
            double ddddS = (1.0 / std::pow(T_takeoff, 4)) * (-360.0 + 720.0 * tau);

            // Z 轴同步上升
            pz = target_z * S;
            vz = target_z * dS;
            az = target_z * ddS;
            jz = target_z * dddS;
            sz = target_z * ddddS;

            // 角速度同步加速
            omega = omega_max * S;
            alpha = omega_max * dS;
            j_theta = omega_max * ddS;
            s_theta = omega_max * dddS;

            // 相位角解析积分 (保证无累积误差)
            double I_S = 2.5 * tau4 - 3.0 * tau5 + tau6;
            theta = omega_max * T_takeoff * I_S; 
        } 
        else if (t < T_takeoff + T_cruise) {
            // ------ 【阶段 2：2圈满速巡航】 ------
            double t_c = t - T_takeoff;
            theta = (circles_takeoff * 2.0 * M_PI) + (omega_max * t_c);
            
            omega = omega_max;
            alpha = 0.0; j_theta = 0.0; s_theta = 0.0;
            
            pz = target_z; vz = 0.0; az = 0.0; jz = 0.0; sz = 0.0;
        } 
        else if (t < T_takeoff + T_cruise + T_land) {
            // ------ 【阶段 3：2圈三维螺旋平滑降落】 ------
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

            // Z 轴同步下降：从 target_z 回归 0
            pz = target_z * (1.0 - S);
            vz = -target_z * dS;
            az = -target_z * ddS;
            jz = -target_z * dddS;
            sz = -target_z * ddddS;

            // 角速度同步衰减：从 omega_max 回归 0
            omega = omega_max * (1.0 - S);
            alpha = -omega_max * dS;
            j_theta = -omega_max * ddS;
            s_theta = -omega_max * dddS;

            // 相位角严谨积分
            double I_1_minus_S = tau - (2.5 * tau4 - 3.0 * tau5 + tau6);
            theta = (circles_takeoff + circles_cruise) * 2.0 * M_PI + (omega_max * T_land * I_1_minus_S); 
        } 
        else {
            // ------ 【阶段 4：落地锁定】 ------
            theta = (circles_takeoff + circles_cruise + circles_land) * 2.0 * M_PI; 
            omega = 0.0; alpha = 0.0; j_theta = 0.0; s_theta = 0.0;
            pz = 0.0; vz = 0.0; az = 0.0; jz = 0.0; sz = 0.0;
        }

        // ================= 严谨的广义极坐标运动学求导 =================
        double c = std::cos(theta);
        double s = std::sin(theta);

        px = R * s;
        py = R - R * c;
        
        vx = R * c * omega;
        vy = R * s * omega;
        
        ax = R * (c * alpha - s * std::pow(omega, 2));
        ay = R * (s * alpha + c * std::pow(omega, 2));
        
        jx = R * (c * (j_theta - std::pow(omega, 3)) - 3.0 * s * omega * alpha);
        jy = R * (s * (j_theta - std::pow(omega, 3)) + 3.0 * c * omega * alpha);
        
        sx = R * (c * (s_theta - 6.0 * std::pow(omega, 2) * alpha) - s * (4.0 * omega * j_theta + 3.0 * std::pow(alpha, 2) - std::pow(omega, 4)));
        sy = R * (s * (s_theta - 6.0 * std::pow(omega, 2) * alpha) + c * (4.0 * omega * j_theta + 3.0 * std::pow(alpha, 2) - std::pow(omega, 4)));

        // 装填标准消息
        msg.position = {(float)px, (float)py, (float)pz};
        msg.velocity = {(float)vx, (float)vy, (float)vz};
        msg.acceleration = {(float)ax, (float)ay, (float)az};
        msg.jerk = {(float)jx, (float)jy, (float)jz};

        // ================= 偏航角解卷绕 =================
// ================= 偏航角指向水平速度方向并解卷绕 =================
        double raw_yaw = 0.0; 
        
        // 1. 获取当前水平速度的模长平方
        double speed_sq = vx * vx + vy * vy;
        
        // 2. 核心数学：如果飞机有水平移动，机头指向速度方向
        if (speed_sq > 1e-6) {
            raw_yaw = std::atan2(vy, vx);
        } else {
            // 3. 奇点保护：如果飞机静止(起飞前或落地后)，提取上一次的偏航角主值
            // 防止 atan2(0,0) 导致机头瞬间抽搐归零
            raw_yaw = first_yaw_ ? 0.0 : std::atan2(std::sin(last_yaw_), std::cos(last_yaw_));
        }
        
        // 4. 偏航角连续性解卷绕 (Angle Unwrapping)
        // 保证机头一直顺着转，而不是到了 180 度突然反转 360 度
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
        msg.yaw = (float)raw_yaw;
        
        double D_val = vx * vx + vy * vy;
        double N_val = vx * ay - vy * ax;
        
        double yawspeed = 0.0;
        double yawaccel = 0.0; 
        
        if (D_val > 1e-4) {
            yawspeed = N_val / D_val;
            double N_dot = vx * jy - vy * jx; 
            double D_dot = 2.0 * vx * ax + 2.0 * vy * ay;
            yawaccel = (N_dot * D_val - N_val * D_dot) / (D_val * D_val);
        }
        msg.yawspeed = (float)yawspeed;

        // ================= 发送数据 =================
        auto timestamp = this->get_clock()->now();
        msg.timestamp = timestamp.nanoseconds() / 1000;
        setpoint_pub_->publish(msg);

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