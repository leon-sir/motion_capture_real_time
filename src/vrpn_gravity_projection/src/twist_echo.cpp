#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    // 原始数据
    double lx = msg->twist.linear.x;
    double ly = msg->twist.linear.y;
    double lz = msg->twist.linear.z;
    
    double ax = msg->twist.angular.x;
    double ay = msg->twist.angular.y;
    double az = msg->twist.angular.z;
    
    // 变换: 第三个取负，然后交换第二和第三
    // (x, y, z) -> (x, -z, y)
    double new_lx = lx;
    double new_ly = -lz;
    double new_lz = ly;
    
    double new_ax = ax;
    double new_ay = -az;
    double new_az = ay;
    
    printf("---\n");
    printf("linear:\n");
    printf("  x: %f\n", new_lx);
    printf("  y: %f\n", new_ly);
    printf("  z: %f\n", new_lz);
    printf("angular:\n");
    printf("  x: %f\n", new_ax);
    printf("  y: %f\n", new_ay);
    printf("  z: %f\n", new_az);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "twist_echo_transformed");
    ros::NodeHandle nh;
    
    printf("Listening to /vrpn_client_node/Rigid/twist ...\n");
    printf("Transform: (x, y, z) -> (x, -z, y)\n");
    
    ros::Subscriber sub = nh.subscribe("/vrpn_client_node/Rigid/twist", 10, twistCallback);
    
    ros::spin();
    return 0;
}
