/**
 * @file twist_transform.cpp
 * @brief ROS节点：将世界坐标系的Twist转换到本体坐标系
 * 
 * 功能：
 * 1. 订阅 VRPN 位姿话题获取姿态
 * 2. 订阅世界坐标系的 Twist 话题
 * 3. 使用四元数将速度转换到本体坐标系
 * 4. 发布本体坐标系的 Twist
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class TwistTransform {
public:
    TwistTransform() : nh_("~"), has_pose_(false) {
        // 从参数服务器加载配置
        nh_.param<std::string>("pose_topic", pose_topic_, "/vrpn_client_node/Rigid/pose");
        nh_.param<std::string>("twist_world_topic", twist_world_topic_, "/twist_world");
        nh_.param<std::string>("twist_body_topic", twist_body_topic_, "/twist_body");
        
        // 订阅位姿话题
        pose_sub_ = nh_.subscribe(pose_topic_, 10, 
                                  &TwistTransform::poseCallback, this);
        
        // 订阅世界坐标系Twist
        twist_world_sub_ = nh_.subscribe(twist_world_topic_, 10, 
                                         &TwistTransform::twistCallback, this);
        
        // 发布本体坐标系Twist
        twist_body_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(twist_body_topic_, 10);
        
        ROS_INFO("Twist Transform Node Started");
        ROS_INFO("  Subscribing pose from: %s", pose_topic_.c_str());
        ROS_INFO("  Subscribing twist (world) from: %s", twist_world_topic_.c_str());
        ROS_INFO("  Publishing twist (body) to: %s", twist_body_topic_.c_str());
    }
    
private:
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 提取四元数 (ROS格式: x, y, z, w)
        double qx = msg->pose.orientation.x;
        double qy = msg->pose.orientation.y;
        double qz = msg->pose.orientation.z;
        double qw = msg->pose.orientation.w;
        
        // 转换为Eigen四元数 (Eigen格式: w, x, y, z)
        current_orientation_ = Eigen::Quaterniond(qw, qx, qy, qz);
        current_orientation_.normalize();
        
        has_pose_ = true;
    }
    
    void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        if (!has_pose_) {
            ROS_WARN_THROTTLE(2.0, "No pose data received yet, cannot transform twist");
            return;
        }
        
        // 提取世界坐标系的线速度和角速度
        Eigen::Vector3d linear_world(msg->twist.linear.x, 
                                      msg->twist.linear.y, 
                                      msg->twist.linear.z);
        
        Eigen::Vector3d angular_world(msg->twist.angular.x, 
                                       msg->twist.angular.y, 
                                       msg->twist.angular.z);
        
        // 转换到本体坐标系
        // v_body = q^(-1) * v_world
        Eigen::Vector3d linear_body = current_orientation_.inverse() * linear_world;
        Eigen::Vector3d angular_body = current_orientation_.inverse() * angular_world;
        
        // 发布本体坐标系Twist
        geometry_msgs::TwistStamped twist_body_msg;
        twist_body_msg.header = msg->header;
        twist_body_msg.header.frame_id = "body";
        
        twist_body_msg.twist.linear.x = linear_body(0);
        twist_body_msg.twist.linear.y = linear_body(1);
        twist_body_msg.twist.linear.z = linear_body(2);
        
        twist_body_msg.twist.angular.x = angular_body(0);
        twist_body_msg.twist.angular.y = angular_body(1);
        twist_body_msg.twist.angular.z = angular_body(2);
        
        twist_body_pub_.publish(twist_body_msg);
    }
    
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Subscriber twist_world_sub_;
    ros::Publisher twist_body_pub_;
    
    std::string pose_topic_;
    std::string twist_world_topic_;
    std::string twist_body_topic_;
    
    Eigen::Quaterniond current_orientation_;
    bool has_pose_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "twist_transform");
    
    TwistTransform node;
    
    ros::spin();
    
    return 0;
}
