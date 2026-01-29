/**
 * @file test_vrpn_gravity.cpp
 * @brief 测试VRPN重力投影计算的正确性
 * 
 * 编译命令：
 * g++ -std=c++14 test_vrpn_gravity.cpp -I/usr/include/eigen3 -o test_vrpn_gravity
 * 
 * 运行：
 * ./test_vrpn_gravity
 */

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

// 模拟VRPN消息结构
struct VRPNPose {
    double position_x, position_y, position_z;
    double orientation_x, orientation_y, orientation_z, orientation_w;
};

// 计算重力投影（与MuJoCo一致）
Eigen::Vector3d compute_projected_gravity(const VRPNPose& pose, 
                                          const Eigen::Vector3d& gravity_world) {
    // ROS四元数格式转Eigen (w, x, y, z)
    Eigen::Quaterniond q(pose.orientation_w, 
                        pose.orientation_x, 
                        pose.orientation_y, 
                        pose.orientation_z);
    
    // 归一化
    q.normalize();
    
    // 计算重力投影
    return q.inverse() * gravity_world;
}

// 测试用例
void test_case(const std::string& name, 
               const VRPNPose& pose,
               const Eigen::Vector3d& expected_gravity) {
    Eigen::Vector3d gravity_world(0.0, 0.0, -9.81);
    Eigen::Vector3d projected = compute_projected_gravity(pose, gravity_world);
    
    double error = (projected - expected_gravity).norm();
    
    std::cout << "\n=== " << name << " ===" << std::endl;
    std::cout << "Quaternion (x,y,z,w): [" 
              << pose.orientation_x << ", "
              << pose.orientation_y << ", "
              << pose.orientation_z << ", "
              << pose.orientation_w << "]" << std::endl;
    std::cout << "Projected Gravity: [" 
              << projected(0) << ", "
              << projected(1) << ", "
              << projected(2) << "]" << std::endl;
    std::cout << "Expected: [" 
              << expected_gravity(0) << ", "
              << expected_gravity(1) << ", "
              << expected_gravity(2) << "]" << std::endl;
    std::cout << "Error: " << error << std::endl;
    
    if (error < 0.01) {
        std::cout << "✓ PASS" << std::endl;
    } else {
        std::cout << "✗ FAIL" << std::endl;
    }
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "VRPN重力投影计算测试" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // 测试1：机器人水平放置（无旋转）
    VRPNPose pose1;
    pose1.position_x = 0.0;
    pose1.position_y = 0.0;
    pose1.position_z = 1.0;
    pose1.orientation_x = 0.0;
    pose1.orientation_y = 0.0;
    pose1.orientation_z = 0.0;
    pose1.orientation_w = 1.0;
    Eigen::Vector3d expected1(0.0, 0.0, -9.81);
    test_case("测试1: 水平放置", pose1, expected1);
    
    // 测试2：绕X轴旋转90度（Roll = 90°）
    VRPNPose pose2;
    pose2.position_x = 0.0;
    pose2.position_y = 0.0;
    pose2.position_z = 1.0;
    double angle = M_PI / 2.0;  // 90度
    pose2.orientation_x = std::sin(angle / 2.0);
    pose2.orientation_y = 0.0;
    pose2.orientation_z = 0.0;
    pose2.orientation_w = std::cos(angle / 2.0);
    Eigen::Vector3d expected2(0.0, 9.81, 0.0);
    test_case("测试2: 绕X轴旋转90度", pose2, expected2);
    
    // 测试3：绕Y轴旋转90度（Pitch = 90°）
    VRPNPose pose3;
    pose3.position_x = 0.0;
    pose3.position_y = 0.0;
    pose3.position_z = 1.0;
    pose3.orientation_x = 0.0;
    pose3.orientation_y = std::sin(angle / 2.0);
    pose3.orientation_z = 0.0;
    pose3.orientation_w = std::cos(angle / 2.0);
    Eigen::Vector3d expected3(-9.81, 0.0, 0.0);
    test_case("测试3: 绕Y轴旋转90度", pose3, expected3);
    
    // 测试4：绕Z轴旋转90度（Yaw = 90°）
    // 绕Z轴旋转不改变重力投影的Z分量
    VRPNPose pose4;
    pose4.position_x = 0.0;
    pose4.position_y = 0.0;
    pose4.position_z = 1.0;
    pose4.orientation_x = 0.0;
    pose4.orientation_y = 0.0;
    pose4.orientation_z = std::sin(angle / 2.0);
    pose4.orientation_w = std::cos(angle / 2.0);
    Eigen::Vector3d expected4(0.0, 0.0, -9.81);
    test_case("测试4: 绕Z轴旋转90度", pose4, expected4);
    
    // 测试5：使用你提供的真实VRPN数据
    VRPNPose pose5;
    pose5.position_x = 0.3288811340332031;
    pose5.position_y = 0.4725716247558594;
    pose5.position_z = 1.1234088134765625;
    pose5.orientation_x = 0.0004507601261138916;
    pose5.orientation_y = -0.0006081538740545511;
    pose5.orientation_z = 0.0005446301074698567;
    pose5.orientation_w = 0.9999996423721313;
    // 由于旋转很小，期望值接近 [0, 0, -9.81]
    Eigen::Vector3d expected5(0.0, 0.0, -9.81);
    test_case("测试5: 真实VRPN数据（小角度）", pose5, expected5);
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "测试完成" << std::endl;
    std::cout << "========================================" << std::endl;
    
    return 0;
}
