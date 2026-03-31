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

        // 专门传递 Snap 和 YawAccel 的扩展通道
        ext_setpoint_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/geometric_controller/traj_ext", 10);

        time_step_ = 0;
        init_visual_path();

        timer_ = this->create_wall_timer(
            10ms, std::bind(&TrajectoryPlanner::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Circular Trajectory Planner (Zero-Start) Started.");
    }

private:
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; 
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ext_setpoint_pub_; 
    rclcpp::TimerBase::SharedPtr timer_;
    
    nav_msgs::msg::Path expected_path_; 
    uint64_t time_step_;

    // ====== 【新增：用于偏航角解卷绕的变量】 ======
    double last_yaw_ = 0.0;
    bool first_yaw_ = true;

    void init_visual_path()
    {
        double R = 3.0;
        double omega = 0.85;
        expected_path_.header.frame_id = "map"; 
        double T = 2.0 * M_PI / omega; 
        double dt = 0.1; 

        // 预先生成平移过的圆形轨迹，用于 RViz 可视化
        for (double t = 0; t <= T; t += dt) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = R * std::sin(omega * t);
            pose.pose.position.y = R - R * std::cos(omega * t); // 圆心在 (0, R)
            pose.pose.position.z = -5.0; 
            expected_path_.poses.push_back(pose);
        }
    }

    void timer_callback()
    {
        px4_msgs::msg::TrajectorySetpoint msg{};
        std_msgs::msg::Float64MultiArray ext_msg{}; 
        ext_msg.data.resize(4); // [snap_x, snap_y, snap_z, yaw_accel]
        
        double t = static_cast<double>(time_step_) * 0.01; 
        double R = 3.0;        
        double omega = 1.25;   

        // ================= Z 轴五阶多项式起飞缓冲 =================
        double target_z = -5.0;     
        double takeoff_time = 5.0;  
        
        double pz, vz, az, jz, sz; 
        
        if (t < takeoff_time) {
            double tau = t / takeoff_time;
            double tau2 = tau * tau;
            double tau3 = tau2 * tau;
            double tau4 = tau3 * tau;
            double tau5 = tau4 * tau;
            
            pz = target_z * (10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5);
            vz = (target_z / takeoff_time) * (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4);
            az = (target_z / (takeoff_time * takeoff_time)) * (60.0 * tau - 180.0 * tau2 + 120.0 * tau3);
            jz = (target_z / (takeoff_time * takeoff_time * takeoff_time)) * (60.0 - 360.0 * tau + 360.0 * tau2);
            sz = (target_z / (std::pow(takeoff_time, 4))) * (-360.0 + 720.0 * tau);
        } else {
            pz = target_z; vz = 0.0; az = 0.0; jz = 0.0; sz = 0.0;
        }

        // ================= X / Y 轴零起点的圆形轨迹严格解析求导 =================
        // 位置方程 (起点为 0,0)
        double px = R * std::sin(omega * t);
        double py = R - R * std::cos(omega * t);
        
        // 速度 (对位置求导)
        double vx = R * omega * std::cos(omega * t);
        double vy = R * omega * std::sin(omega * t);
        
        // 加速度 (对速度求导)
        double ax = -R * std::pow(omega, 2) * std::sin(omega * t);
        double ay =  R * std::pow(omega, 2) * std::cos(omega * t);
        
        // 加加速度 Jerk (对加速度求导)
        double jx = -R * std::pow(omega, 3) * std::cos(omega * t);
        double jy = -R * std::pow(omega, 3) * std::sin(omega * t);

        // 四阶导数 Snap (对 Jerk 求导)
        double sx =  R * std::pow(omega, 4) * std::sin(omega * t);
        double sy = -R * std::pow(omega, 4) * std::cos(omega * t);

        // 装填标准消息
        msg.position = {(float)px, (float)py, (float)pz};
        msg.velocity = {(float)vx, (float)vy, (float)vz};
        msg.acceleration = {(float)ax, (float)ay, (float)az};
        msg.jerk = {(float)jx, (float)jy, (float)jz};

        // ================= 偏航角与偏航角加速度严格解析 =================
        double raw_yaw = 0;//std::atan2(vy, vx);
        
        // ====== 【新增：偏航角解卷绕 (Angle Unwrapping)】 ======
        if (first_yaw_) {
            last_yaw_ = raw_yaw;
            first_yaw_ = false;
        } else {
            double dyaw = raw_yaw - last_yaw_;
            // 强制将差值约束在 [-pi, pi] 范围内
            while (dyaw > M_PI)  dyaw -= 2.0 * M_PI;
            while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
            
            // 累加平滑后的偏航角
            raw_yaw = last_yaw_ + dyaw;
            last_yaw_ = raw_yaw;
        }
        msg.yaw = (float)raw_yaw;
        // =========================================================
        
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

        // 装填并发送扩展数据
        ext_msg.data[0] = sx;
        ext_msg.data[1] = sy;
        ext_msg.data[2] = sz;
        ext_msg.data[3] = yawaccel;
        ext_setpoint_pub_->publish(ext_msg);

        // 发布 RViz 路径
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