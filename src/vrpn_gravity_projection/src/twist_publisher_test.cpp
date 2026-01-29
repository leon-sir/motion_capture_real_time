#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "twist_publisher_test");
    ros::NodeHandle nh;
    
    ros::Publisher pub = nh.advertise<geometry_msgs::TwistStamped>("/twist_body", 10);
    ros::Rate rate(50); // 50Hz
    
    ROS_INFO("Twist Publisher Test Node Started");
    
    while(ros::ok()) {
        geometry_msgs::TwistStamped twist_msg;
        twist_msg.header.stamp = ros::Time::now();
        twist_msg.header.frame_id = "body";
        
        // 设置一些测试数据
        twist_msg.twist.linear.x = 0.1 * sin(ros::Time::now().toSec());  // 线速度x方向
        twist_msg.twist.linear.y = 0.05 * cos(ros::Time::now().toSec()); // 线速度y方向
        twist_msg.twist.linear.z = 0.0;                                  // 线速度z方向
        
        // 设置角速度
        twist_msg.twist.angular.x = 0.2 * sin(ros::Time::now().toSec() * 2.0); // 绕x轴旋转
        twist_msg.twist.angular.y = 0.1 * cos(ros::Time::now().toSec() * 1.5); // 绕y轴旋转
        twist_msg.twist.angular.z = 0.15 * sin(ros::Time::now().toSec() * 1.2); // 绕z轴旋转
        
        pub.publish(twist_msg);
        
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}