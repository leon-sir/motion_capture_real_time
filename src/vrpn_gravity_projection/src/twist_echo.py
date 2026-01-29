#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
订阅 /vrpn_client_node/Rigid/twist 话题
对数据进行变换：第三个取负，然后交换第二和第三个位置
即: (x, y, z) -> (x, -z, y)
"""

import rospy
from geometry_msgs.msg import TwistStamped

def twist_callback(msg):
    # 原始数据
    lx = msg.twist.linear.x
    ly = msg.twist.linear.y
    lz = msg.twist.linear.z
    
    ax = msg.twist.angular.x
    ay = msg.twist.angular.y
    az = msg.twist.angular.z
    
    # 变换: 第三个取负，然后交换第二和第三
    # (x, y, z) -> (x, -z, y)
    new_lx = lx
    new_ly = -lz
    new_lz = ly
    
    new_ax = ax
    new_ay = -az
    new_az = ay
    
    print("---")
    print("linear:")
    print("  x: %.6f" % new_lx)
    print("  y: %.6f" % new_ly)
    print("  z: %.6f" % new_lz)
    print("angular:")
    print("  x: %.6f" % new_ax)
    print("  y: %.6f" % new_ay)
    print("  z: %.6f" % new_az)

if __name__ == '__main__':
    rospy.init_node('twist_echo_transformed', anonymous=True)
    rospy.Subscriber('/vrpn_client_node/Rigid/twist', TwistStamped, twist_callback)
    print("Listening to /vrpn_client_node/Rigid/twist ...")
    print("Transform: (x, y, z) -> (x, -z, y)")
    rospy.spin()
