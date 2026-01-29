/**
 * @file vrpn_gravity_projection.cpp
 * @brief ROS节点：重力投影 + 线速度计算 + 外部平滑角速度转换 (修复日志打印版)
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

class VRPNGravityProjection {
public:
    VRPNGravityProjection() : nh_("~") {
        // --- 1. 基础配置 ---
        nh_.param<std::string>("vrpn_topic", vrpn_topic_, "/vrpn_client_node/Rigid/pose");
        nh_.param<std::string>("gravity_topic", gravity_topic_, "/projected_gravity");
        std::vector<double> gravity_vec;
        nh_.param<std::vector<double>>("gravity_vector", gravity_vec, {0.0, 0.0, -1});
        if (gravity_vec.size() == 3) {
            gravity_ << gravity_vec[0], gravity_vec[1], gravity_vec[2];
        } else {
            gravity_ << 0.0, 0.0, -1;
        }
        nh_.param<bool>("debug", debug_output_, false);

        // --- 2. 线速度配置 ---
        nh_.param<bool>("enable_velocity", enable_velocity_, false);
        nh_.param<std::string>("velocity_topic", velocity_topic_, "/projected_velocity");
        nh_.param<std::string>("velocity_source", velocity_source_, "pose_diff"); // "pose_diff" or "twist"
        nh_.param<std::string>("twist_topic", twist_topic_, "/vrpn_client_node/Rigid/twist");

        // --- 3. 角速度配置 ---
        nh_.param<bool>("enable_ang_velocity", enable_ang_velocity_, false);
        nh_.param<std::string>("ang_velocity_topic", ang_velocity_topic_, "/projected_omega");
        nh_.param<std::string>("ang_velocity_source", ang_velocity_source_, "external_smooth"); 
        nh_.param<std::string>("external_ang_vel_topic", external_ang_vel_topic_, "/vrpn_client_node/Rigid/angular_velocity");

        // --- 4. 初始化订阅者和发布者 ---
        
        // 订阅 VRPN 位姿 (核心)
        vrpn_sub_ = nh_.subscribe(vrpn_topic_, 10, &VRPNGravityProjection::vrpnCallback, this);
        
        // 发布重力
        gravity_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(gravity_topic_, 10);

        // 发布线速度
        if (enable_velocity_) {
            velocity_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(velocity_topic_, 10);
            
            if (velocity_source_ == "twist") {
                twist_sub_ = nh_.subscribe(twist_topic_, 10, &VRPNGravityProjection::twistCallback, this);
                ROS_INFO("Subscribed to twist source: %s", twist_topic_.c_str());
            }
        }

        // 发布角速度
        if (enable_ang_velocity_) {
            angular_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(ang_velocity_topic_, 10);
            
            if (ang_velocity_source_ == "external_smooth") {
                ext_ang_vel_sub_ = nh_.subscribe(external_ang_vel_topic_, 10, &VRPNGravityProjection::externalAngVelCallback, this);
                ROS_INFO("Subscribing to smoothed angular velocity: %s", external_ang_vel_topic_.c_str());
            }
        }

        last_msg_time_ = ros::Time::now();
        ROS_INFO("VRPN Projection Node Started (Logging Fixed).");
    }

private:
    // --- 回调 1: 处理位姿 (重力 + 线速度差分) ---
    void vrpnCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        last_msg_time_ = ros::Time::now();

        // 1. 提取四元数
        Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
        q.normalize();

        last_q_ = q;
        has_last_q_ = true;

        // 2. 处理重力投影
        Eigen::Vector3d gravity_mocap(0.0, -1.0, 0.0); // Y-up
        Eigen::Vector3d proj_grav_body = q.inverse() * gravity_mocap;
        
        geometry_msgs::Vector3Stamped grav_msg;
        grav_msg.header = msg->header;
        grav_msg.vector.x = proj_grav_body(0);
        grav_msg.vector.y = -proj_grav_body(2); // (x, -z, y)
        grav_msg.vector.z = proj_grav_body(1);
        gravity_pub_.publish(grav_msg);

        // [修复] 打印重力信息
        if (debug_output_) {
            ROS_INFO_THROTTLE(1.0, "Projected Gravity: [%.4f, %.4f, %.4f]", 
                              grav_msg.vector.x, grav_msg.vector.y, grav_msg.vector.z);
        }

        // 3. 处理线速度 (如果是 pose_diff 模式)
        if (enable_velocity_ && velocity_source_ == "pose_diff") {
            Eigen::Vector3d curr_pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
            ros::Time curr_time = msg->header.stamp;

            if (has_last_position_) {
                double dt = (curr_time - last_pos_time_).toSec();
                if (dt > 1e-6) {
                    Eigen::Vector3d v_world = (curr_pos - last_position_) / dt;
                    Eigen::Vector3d v_body = q.inverse() * v_world;

                    geometry_msgs::Vector3Stamped vel_msg;
                    vel_msg.header = msg->header;
                    vel_msg.vector.x = v_body(0);
                    vel_msg.vector.y = -v_body(2); // (x, -z, y)
                    vel_msg.vector.z = v_body(1);
                    velocity_pub_.publish(vel_msg);

                    // [修复] 打印 pose_diff 速度
                    if (debug_output_) {
                        ROS_INFO_THROTTLE(1.0, "Projected Velocity (pose_diff): [%.4f, %.4f, %.4f]", 
                                          vel_msg.vector.x, vel_msg.vector.y, vel_msg.vector.z);
                    }
                }
            }
            last_position_ = curr_pos;
            last_pos_time_ = curr_time;
            has_last_position_ = true;
        }
    }

    // --- 回调 2: 处理外部平滑角速度 ---
    void externalAngVelCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
        if (!enable_ang_velocity_ || !has_last_q_) return;

        Eigen::Vector3d omega_world(msg->vector.x, msg->vector.y, msg->vector.z);
        Eigen::Vector3d omega_body = last_q_.inverse() * omega_world;

        geometry_msgs::Vector3Stamped out_msg;
        out_msg.header = msg->header;
        out_msg.vector.x = omega_body(0);
        out_msg.vector.y = -omega_body(2); // (x, -z, y)
        out_msg.vector.z = omega_body(1);

        angular_pub_.publish(out_msg);

        // 打印平滑后的角速度
        if (debug_output_) {
            ROS_INFO_THROTTLE(1.0, "Smoothed Omega Body: [%.4f, %.4f, %.4f]", 
                              out_msg.vector.x, out_msg.vector.y, out_msg.vector.z);
        }
    }

    // --- 回调 3: 处理 Twist 线速度 ---
    void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        if (enable_velocity_ && velocity_source_ == "twist" && has_last_q_) {
            Eigen::Vector3d v_world(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
            Eigen::Vector3d v_body = last_q_.inverse() * v_world;

            geometry_msgs::Vector3Stamped vel_msg;
            vel_msg.header = msg->header;
            vel_msg.vector.x = v_body(0);
            vel_msg.vector.y = -v_body(2); // (x, -z, y)
            vel_msg.vector.z = v_body(1);
            velocity_pub_.publish(vel_msg);

            // [修复] 打印 twist 速度
            if (debug_output_) {
                ROS_INFO_THROTTLE(1.0, "Projected Velocity (twist input): [%.4f, %.4f, %.4f]", 
                                  vel_msg.vector.x, vel_msg.vector.y, vel_msg.vector.z);
            }
        }
    }

    // --- 成员变量 ---
    ros::NodeHandle nh_;
    ros::Subscriber vrpn_sub_;
    ros::Subscriber twist_sub_;
    ros::Subscriber ext_ang_vel_sub_;

    ros::Publisher gravity_pub_;
    ros::Publisher velocity_pub_;
    ros::Publisher angular_pub_;

    std::string vrpn_topic_;
    std::string gravity_topic_;
    Eigen::Vector3d gravity_;
    
    // 线速度相关
    bool enable_velocity_;
    std::string velocity_topic_;
    std::string velocity_source_;
    std::string twist_topic_;
    Eigen::Vector3d last_position_;
    ros::Time last_pos_time_;
    bool has_last_position_ = false;

    // 角速度相关
    bool enable_ang_velocity_;
    std::string ang_velocity_topic_;
    std::string ang_velocity_source_;
    std::string external_ang_vel_topic_;

    // 共享状态
    Eigen::Quaterniond last_q_;
    bool has_last_q_ = false;
    ros::Time last_msg_time_;
    bool debug_output_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "vrpn_gravity_projection");
    VRPNGravityProjection node;
    ros::spin();
    return 0;
}