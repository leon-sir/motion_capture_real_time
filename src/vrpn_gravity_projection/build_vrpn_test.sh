#!/bin/bash

# 编译VRPN重力投影测试程序

echo "编译 test_vrpn_gravity..."
g++ -std=c++14 test/test_vrpn_gravity.cpp -I/usr/include/eigen3 -o test_vrpn_gravity

if [ $? -eq 0 ]; then
    echo "✓ 编译成功！"
    echo ""
    echo "运行测试："
    echo "./test_vrpn_gravity"
    echo ""
    echo "或者直接运行："
    ./test_vrpn_gravity
else
    echo "✗ 编译失败"
    echo "请确保已安装 Eigen3："
    echo "  sudo apt-get install libeigen3-dev"
fi
