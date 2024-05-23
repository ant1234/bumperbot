
#ifndef Noisy_CONTROLLER_HPP
#define Noisy_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class NoisyController : public rclcpp::Node
{
    public:
        NoisyController(const std::string & name);

    private:
        void jointCallback(const sensor_msgs::msg::JointState & msg);

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

        double wheel_radius_;
        double wheel_separation_;

        double left_wheel_previous_position_;
        double right_wheel_previous_position_;

        rclcpp::Time previous_time_;

        double x_;
        double y_;
        double theta_;

        nav_msgs::msg::Odometry odom_msg_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
        geometry_msgs::msg::TransformStamped transform_stamped_;

};

#endif