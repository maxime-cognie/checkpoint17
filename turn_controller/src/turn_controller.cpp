#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/impl/utils.h"

#include <Eigen/Dense>

#include <cmath>
#include <cstddef>
#include <functional>
#include <thread>

using namespace std::chrono_literals;

class TurnController: 
    public rclcpp::Node {

public:   
    TurnController(int scene_number):
        Node("turn_controller_node"),
        kP_(0.2), kD_(0.2), kI_(0.01),
        rate(100ms),
        scene_number_(scene_number){
        using std::placeholders::_1;

        timer_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_ = this->create_wall_timer(
            1s,
            std::bind(&TurnController::timer_callback, this),
            timer_callback_group_);

        odom_sub_options_.callback_group = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/filtered", 10,
            std::bind(&TurnController::odom_callback, this, _1),
            odom_sub_options_);

        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", 1);
        
        current_pose_ << 0.0, 0.0, 0.0;
        set_waypoints();
    }

private:
    void timer_callback() {
        timer_->cancel();
        geometry_msgs::msg::Twist cmd_vel;

        float err_orientation, prev_orientation, waypoint_orientation;
        float sum_I;
        float X_dot;
        float input;

        rclcpp::Time current_time = this->get_clock()->now(),
        prev_time = this->get_clock()->now();
        float dt;

        for(size_t i = 0; i < waypoints_.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "rotating toward waypoint %ld", i+1);
            waypoint_orientation = std::atan2(
                    waypoints_[i](1) - current_pose_(1),
                    waypoints_[i](0) - current_pose_(0));
            err_orientation = waypoint_orientation - current_pose_(2);
            prev_orientation = current_pose_(2);
            sum_I = 0.0;
            X_dot = 0.0;

            
            while (std::abs(err_orientation) >= 0.02 && rclcpp::ok()) {
                err_orientation = waypoint_orientation - current_pose_(2);
                current_time = this->get_clock()->now();
                dt = (current_time - prev_time).seconds();
                sum_I += err_orientation * dt;
                X_dot = (current_pose_(2) - prev_orientation)/dt;

                input = kP_*err_orientation + kI_*sum_I + kD_*X_dot;
                RCLCPP_DEBUG(this->get_logger(), "err_orientation: %f",err_orientation);
                
                cmd_vel.angular.z = input;
                twist_pub_->publish(cmd_vel);

                prev_orientation= current_pose_(2);
                prev_time = current_time;
                rate.sleep();
            }
            cmd_vel.angular.z = 0;
            twist_pub_->publish(cmd_vel);
            RCLCPP_INFO(this->get_logger(), "Waypoint %ld reached", i+1);
            std::this_thread::sleep_for(1.5s);
        }
        RCLCPP_INFO(this->get_logger(), "Mission done!");
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_(0) = msg->pose.pose.position.x;
        current_pose_(1) = msg->pose.pose.position.y;
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        current_pose_(2) = tf2::impl::getYaw(q);
    }

    void set_waypoints() {
        switch (scene_number_) {
            case 1: // Simulation
            waypoints_ = {
                {0.6, -1.4},
                {1.40, -0.25},
                {0.8, 0.55}};
            break;

            case 2: // CyberWorld
            waypoints_ = {
                {1.0, -0.5},
                {0.5, -1.05}};
            break;

            default:
            RCLCPP_ERROR(this->get_logger(), "Invalid scene number: %d",
                        scene_number_);
            return;
        }
    }

    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::SubscriptionOptions odom_sub_options_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Rate rate;

    Eigen::Vector3f current_pose_;
    std::vector<Eigen::Vector2f> waypoints_{
        {0.6, -1.4},
        {1.40, -0.25},
        {0.8, 0.55}};
    float kP_, kI_, kD_;

    int scene_number_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  int scene_number = 1; // Default scene number
  if (argc > 1) {
    scene_number = std::atoi(argv[1]);
  }

  auto turn_controller = std::make_shared<TurnController>(scene_number);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(turn_controller);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
