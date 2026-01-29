#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped, Twist

def twist_callback(msg):
    new_msg = TwistStamped()
    new_msg.header = msg.header
    
    # 假设你要变换的是 linear 部分 (x, y, z)
    # 原始: x, y, z
    # 变换: x, -z, y (第三个取负，然后交换第二和第三)
    new_msg.twist.linear.x = msg.twist.linear.x
    new_msg.twist.linear.y = -msg.twist.linear.z  # 第三个取负后放到第二位
    new_msg.twist.linear.z = msg.twist.linear.y   # 原第二个放到第三位
    
    # angular 部分同样处理
    new_msg.twist.angular.x = msg.twist.angular.x
    new_msg.twist.angular.y = -msg.twist.angular.z
    new_msg.twist.angular.z = msg.twist.angular.y
    
    pub.publish(new_msg)

if __name__ == '__main__':
    rospy.init_node('twist_transformer')
    pub = rospy.Publisher('/vrpn_client_node/Rigid/twist_transformed', TwistStamped, queue_size=10)
    rospy.Subscriber('/vrpn_client_node/Rigid/twist', TwistStamped, twist_callback)
    rospy.spin()
