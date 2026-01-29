/**
 * @file reset_four_element.cpp
 * @brief 50Hz 坐标转换集成节点
 * 功能：VRPN(Y-up) -> ROS(Z-up) 转换、重力投影、线速度、角速度计算、CSV保存及Topic发布
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <string>

class ResetFourElement {
public:
    ResetFourElement() : nh_("~") {
        // --- 1. 参数加载 ---
        nh_.param<std::string>("vrpn_topic", vrpn_topic_, "/vrpn_client_node/Rigid/pose");
        nh_.param<bool>("debug", debug_output_, true);
        nh_.param<double>("lpf_alpha_omega", lpf_alpha_omega_, 0.15);
        nh_.param<double>("lpf_alpha_vel", lpf_alpha_vel_, 0.2);

        // --- 2. 初始化 CSV 文件 (带时间戳) ---
        initCSV();

        // --- 3. 定义发布者 ---
        gravity_pub_  = nh_.advertise<geometry_msgs::Vector3Stamped>("/projected_gravity", 10);
        velocity_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/projected_velocity", 10);
        angular_pub_  = nh_.advertise<geometry_msgs::Vector3Stamped>("/projected_omega", 10);

        // --- 4. 订阅与定时器 (50Hz) ---
        vrpn_sub_ = nh_.subscribe(vrpn_topic_, 10, &ResetFourElement::vrpnCallback, this);
        timer_    = nh_.createTimer(ros::Duration(0.02), &ResetFourElement::timerCallback, this);

        // 变量初始化
        last_omega_body_.setZero();
        last_vel_body_.setZero();
        has_data_ = false;
        has_last_state_ = false;

        ROS_INFO("VRPN 50Hz Integrated Node Started.");
        ROS_INFO("Publishing: /projected_gravity, /projected_velocity, /projected_omega");
    }

    ~ResetFourElement() {
        if (csv_grav_.is_open())  csv_grav_.close();
        if (csv_vel_.is_open())   csv_vel_.close();
        if (csv_omega_.is_open()) csv_omega_.close();
        ROS_INFO("All CSV logs closed.");
    }

private:
    void initCSV() {
        std::time_t now = std::time(nullptr);
        char ts[20];
        std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&now));
        std::string time_suffix(ts);

        std::string folder = std::string(getenv("HOME")) + "/mocap_data/";
        system(("mkdir -p " + folder).c_str());

        csv_grav_.open(folder + "gravity_" + time_suffix + ".csv");
        csv_vel_.open(folder + "velocity_" + time_suffix + ".csv");
        csv_omega_.open(folder + "omega_" + time_suffix + ".csv");

        if (csv_grav_.is_open())  csv_grav_ << "ros_time,gx,gy,gz\n";
        if (csv_vel_.is_open())   csv_vel_ << "ros_time,vx,vy,vz\n";
        if (csv_omega_.is_open()) csv_omega_ << "ros_time,wx,wy,wz\n";
        
        ROS_INFO("CSV Logging to: %s", folder.c_str());
    }

    void vrpnCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // --- 核心转换：Y-up 到 Z-up ---
        // 位置转换
        curr_pos_zup_ = Eigen::Vector3d(msg->pose.position.x, -msg->pose.position.z, msg->pose.position.y);
        
        // 四元数转换 (w, x, -z, y)
        Eigen::Quaterniond q_raw(msg->pose.orientation.w, msg->pose.orientation.x, 
                                 msg->pose.orientation.y, msg->pose.orientation.z);
        q_zup_ = Eigen::Quaterniond(q_raw.w(), q_raw.x(), -q_raw.z(), q_raw.y());
        q_zup_.normalize();

        header_ = msg->header;
        has_data_ = true;
    }

    void timerCallback(const ros::TimerEvent& event) {
        if (!has_data_) return;

        double dt = 0.02; // 固定的 50Hz 步长
        double ros_ts = event.current_real.toSec();

        // 1. 重力投影 (Body frame)
        Eigen::Vector3d gravity_world(0, 0, -1);
        Eigen::Vector3d proj_grav = q_zup_.inverse() * gravity_world;
        publishVector(gravity_pub_, header_, proj_grav);
        writeCSV(csv_grav_, ros_ts, proj_grav);

        if (has_last_state_) {
            // 2. 线速度计算 (Body frame)
            Eigen::Vector3d v_world = (curr_pos_zup_ - last_pos_zup_) / dt;
            Eigen::Vector3d v_body_raw = q_zup_.inverse() * v_world;
            last_vel_body_ = lpf_alpha_vel_ * v_body_raw + (1.0 - lpf_alpha_vel_) * last_vel_body_;
            publishVector(velocity_pub_, header_, last_vel_body_);
            writeCSV(csv_vel_, ros_ts, last_vel_body_);

            // 3. 角速度计算 (Body frame)
            if (q_zup_.coeffs().dot(last_q_zup_.coeffs()) < 0.0) {
                last_q_zup_.coeffs() = -last_q_zup_.coeffs();
            }
            Eigen::Quaterniond dq;
            dq.coeffs() = (q_zup_.coeffs() - last_q_zup_.coeffs()) / dt;
            Eigen::Quaterniond omega_quat = q_zup_.inverse() * dq;
            Eigen::Vector3d omega_body_raw(2.0 * omega_quat.x(), 2.0 * omega_quat.y(), 2.0 * omega_quat.z());
            last_omega_body_ = lpf_alpha_omega_ * omega_body_raw + (1.0 - lpf_alpha_omega_) * last_omega_body_;
            publishVector(angular_pub_, header_, last_omega_body_);
            writeCSV(csv_omega_, ros_ts, last_omega_body_);
        }

        if (debug_output_ && has_last_state_) {
            // 设置为 1.0 代表每秒打印一次详细数据，如果你想每 0.02s 都打印（不建议，会卡顿），可以去掉 _THROTTLE
            ROS_INFO_THROTTLE(1.0, "[50Hz] Grav: [%.4f, %.4f, %.4f] | Vel: [%.4f, %.4f, %.4f] | Omega: [%.4f, %.4f, %.4f]", 
                            proj_grav.x(), proj_grav.y(), proj_grav.z(),
                            last_vel_body_.x(), last_vel_body_.y(), last_vel_body_.z(),
                            last_omega_body_.x(), last_omega_body_.y(), last_omega_body_.z());
        }

        last_pos_zup_ = curr_pos_zup_;
        last_q_zup_ = q_zup_;
        has_last_state_ = true;
    }

    void publishVector(ros::Publisher& pub, const std_msgs::Header& h, const Eigen::Vector3d& v) {
        geometry_msgs::Vector3Stamped m;
        m.header = h;
        m.vector.x = v.x(); m.vector.y = v.y(); m.vector.z = v.z();
        pub.publish(m);
    }

    void writeCSV(std::ofstream& file, double ts, const Eigen::Vector3d& v) {
        if (file.is_open()) {
            file << std::fixed << std::setprecision(6) << ts << "," 
                 << v.x() << "," << v.y() << "," << v.z() << "\n";
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber vrpn_sub_;
    ros::Timer timer_;
    ros::Publisher gravity_pub_, velocity_pub_, angular_pub_;
    std::ofstream csv_grav_, csv_vel_, csv_omega_;

    std::string vrpn_topic_;
    std_msgs::Header header_;
    bool has_data_, has_last_state_, debug_output_;
    Eigen::Vector3d curr_pos_zup_, last_pos_zup_, last_omega_body_, last_vel_body_;
    Eigen::Quaterniond q_zup_, last_q_zup_;
    double lpf_alpha_omega_, lpf_alpha_vel_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "reset_four_element");
    ResetFourElement node;
    ros::spin();
    return 0;
}