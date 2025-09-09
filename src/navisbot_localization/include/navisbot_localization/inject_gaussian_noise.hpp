#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>


class InjectGaussianNoise: public rclcpp::Node
{
public:
    InjectGaussianNoise(const std::string& name);

private:

    // Odometry
    double left_wheel_prev_pos_;
    double right_wheel_prev_pos_;
    double x_;
    double y_;
    double theta_;
    double wheel_radius_;
    double wheel_separation_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr noisy_joint_pub_;

    void jointCallback(const sensor_msgs::msg::JointState &msg);
    
    
    rclcpp::Time prev_time_;
    nav_msgs::msg::Odometry odom_msg_;
    

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
    geometry_msgs::msg::TransformStamped transform_stamped_;
};

