#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <deque>

class SlidingWindowVelocityEstimator {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_vel_;

    // 缓存窗口
    std::deque<geometry_msgs::Quaternion> q_buffer_;
    std::deque<ros::Time> t_buffer_;
    const size_t window_size_ = 20; // 跨度为20帧

public:
    SlidingWindowVelocityEstimator() {
        sub_ = nh_.subscribe("/vrpn_client_node/Rigid/pose", 10, &SlidingWindowVelocityEstimator::poseCallback, this);
        pub_vel_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/vrpn_client_node/Rigid/angular_velocity", 10);
        ROS_INFO("Sliding Window Estimator (Window: 20) Started.");
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 1. 将当前数据压入队列
        q_buffer_.push_back(msg->pose.orientation);
        t_buffer_.push_back(msg->header.stamp);

        // 2. 只有当队列存够了 window_size_ 帧才开始计算
        if (q_buffer_.size() >= window_size_) {
            
            // 获取当前帧 (末尾)
            geometry_msgs::Quaternion q_curr_msg = q_buffer_.back();
            ros::Time t_curr = t_buffer_.back();

            // 获取历史帧 (开头)
            geometry_msgs::Quaternion q_old_msg = q_buffer_.front();
            ros::Time t_old = t_buffer_.front();

            double dt = (t_curr - t_old).toSec();

            if (dt > 0.05) { // 确保时间跨度足够大
                tf2::Quaternion q_curr(q_curr_msg.x, q_curr_msg.y, q_curr_msg.z, q_curr_msg.w);
                tf2::Quaternion q_old(q_old_msg.x, q_old_msg.y, q_old_msg.z, q_old_msg.w);

                // 处理四元数突变 (q 和 -q)
                if (q_curr.dot(q_old) < 0) {
                    q_curr = tf2::Quaternion(-q_curr.x(), -q_curr.y(), -q_curr.z(), -q_curr.w());
                }

                // 计算这 20 帧期间的总旋转差
                // q_diff = q_curr * q_old.inverse()
                tf2::Quaternion q_diff = q_curr * q_old.inverse();

                // 这里的 dt 是 20 帧的总时长，噪声被稀释了 20 倍
                double wx = 2.0 * q_diff.x() / dt;
                double wy = 2.0 * q_diff.y() / dt;
                double wz = 2.0 * q_diff.z() / dt;

                // 发布平滑后的角速度
                geometry_msgs::Vector3Stamped vel_msg;
                vel_msg.header = msg->header;
                vel_msg.vector.x = wx;
                vel_msg.vector.y = wy;
                vel_msg.vector.z = wz;
                pub_vel_.publish(vel_msg);

                // 每 20 帧打印一次
                static int count = 0;
                if (++count % 20 == 0) {
                    ROS_INFO("Win_dt: %.3fs | Raw Vel: [%.3f, %.3f, %.3f]", dt, wx, wy, wz);
                }
            }

            // 弹出最旧的一帧，保持窗口长度
            q_buffer_.pop_front();
            t_buffer_.pop_front();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sliding_window_node");
    SlidingWindowVelocityEstimator estimator;
    ros::spin();
    return 0;
}