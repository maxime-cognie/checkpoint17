#include "rclcpp/callback_group.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/utilities.hpp"
#include "tf2/impl/utils.h"
#include <Eigen/Dense>

#include <cstddef>
#include <functional>

using namespace std::chrono_literals;

class DistanceController: 
    public rclcpp::Node {

public:   
    DistanceController():
        Node("distance_controller_node"),
        kP_(0.3), kD_(1.0), kI_(0.01){
        using std::placeholders::_1;

        timer_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_ = this->create_wall_timer(
            1s,
            std::bind(&DistanceController::timer_callback, this),
            timer_callback_group_);

        odom_sub_options_.callback_group = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/filtered", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                current_pose_(0) = msg->pose.pose.position.x;
                current_pose_(1) = msg->pose.pose.position.y;},
            odom_sub_options_);

        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", 1);
        
        current_pose_ << 0.0, 0.0;
    }

private:
    void timer_callback() {
        timer_->cancel();

        rclcpp::Rate rate(100ms);
        geometry_msgs::msg::Twist cmd_vel;
        Eigen::Vector2f err_pose;
        Eigen::Vector2f prev_pose{0.0, 0.0};
        Eigen::Vector2f sum_I;
        Eigen::Vector2f X_dot;
        Eigen::Vector2f input, U, U_prev;
        U_prev << 0.0, 0.0;
        rclcpp::Time current_time = this->get_clock()->now(),
        prev_time = this->get_clock()->now();
        float dt;

        for(size_t i = 0; i < 3; i++) {
            RCLCPP_INFO(this->get_logger(), "Moving to waypoint %ld", i+1);
            err_pose = waypoints_[i] - current_pose_;
            sum_I << 0.0, 0.0;
            X_dot << 0.0, 0.0;
            while (err_pose.norm() >= 0.02 && rclcpp::ok()) {
                err_pose = waypoints_[i] - current_pose_;
                current_time = this->get_clock()->now();
                dt = (current_time - prev_time).seconds();
                sum_I += err_pose * dt;
                X_dot = (current_pose_ - prev_pose)/dt;

                U = kP_*err_pose + kI_*sum_I + kD_*X_dot;
                input = U;
                RCLCPP_DEBUG(this->get_logger(), "err_pose: %f, %f",err_pose(0), err_pose(1));

                cmd_vel.linear.x = input(0);
                cmd_vel.linear.y = input(1);
                twist_pub_->publish(cmd_vel);

                prev_pose = current_pose_;
                prev_time = current_time;
                U_prev = U;
                rate.sleep();
            }
        }
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        twist_pub_->publish(cmd_vel);
        RCLCPP_INFO(this->get_logger(), "Mission done!");
    }

    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::SubscriptionOptions odom_sub_options_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

    Eigen::Vector2f current_pose_;
    std::vector<Eigen::Vector2f> waypoints_{
        {1.0, 0.0},
        {2.0, 0.0},
        {3.0, 0.0}};
    float kP_, kI_, kD_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto distance_controller = std::make_shared<DistanceController>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(distance_controller);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

//     void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
//         current_pose_(0) = msg->pose.pose.position.x;
//         current_pose_(1) = msg->pose.pose.position.y;
//     }