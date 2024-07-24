#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/time.hpp"
#include "tf2/impl/utils.h"
#include "ament_index_cpp/get_package_share_directory.hpp" // Include this header
#include "yaml-cpp/yaml.h" // include the yaml library
#include <filesystem> // Include the filesystem library
#include <Eigen/Dense>

#include <cstddef>
#include <functional>
#include <thread>

using namespace std::chrono_literals;

class PIDMazeSolver: 
    public rclcpp::Node {

public:   
    PIDMazeSolver(int scene_number):
        Node("distance_controller_node"),
        kP_pose_(1.0), kD_pose_(0.3), kI_pose_(0.01),
        kP_orientation_(0.8), kD_orientation_(0.1), kI_orientation_(0.04),
        scene_number_(scene_number){
        using std::placeholders::_1;

        timer_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_ = this->create_wall_timer(
            1s,
            std::bind(&PIDMazeSolver::timer_callback, this),
            timer_callback_group_);

        odom_sub_options_.callback_group = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/filtered", 10,
            std::bind(&PIDMazeSolver::odom_callback, this, _1),
            odom_sub_options_);

        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", 1);
        
        current_pose_ << 0.0, 0.0;
        waypoints_ = readPointsFromYAML();

        if (scene_number > 1) {
            kP_pose_ = 0.15;
            kI_pose_ = 0.002;
            kD_pose_ = 0.1;
            kP_orientation_ = 0.25;
            kD_orientation_ = 0.1;
            kI_orientation_ = 0.001;
        }
    }

private:
    void timer_callback() {
        timer_->cancel();

        rclcpp::Time starting_time;
        float time_elapsed;
        starting_time = this->get_clock()->now();

        /*  For all the waypoints: 
            - Align the robot with the waypoint
            - Move forward until the robot reaches the waypoint
        */
        for(size_t i = 0; i < waypoints_.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "Moving to waypoint %ld", i+1);
            turn(waypoints_[i]);
            move_forward(waypoints_[i]);
            RCLCPP_INFO(this->get_logger(), "Waypoint %ld reached", i+1);
        }
        // Display the time took by the robot to solve the maze
        time_elapsed = (this->get_clock()->now() - starting_time).seconds();
        RCLCPP_INFO(this->get_logger(), "Mission done in %.1f s!", time_elapsed);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_(0) = msg->pose.pose.position.x;
        current_pose_(1) = msg->pose.pose.position.y;
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        orientation_ = tf2::impl::getYaw(q);
    }

    void move_forward(const Eigen::Vector2f wp){
        float err_pose = (wp - current_pose_).norm();
        Eigen::Vector2f prev_pose = current_pose_;
        float sum_I(0.0);
        float X_dot(0.0);
        float input;
        
        rclcpp::Time current_time = this->get_clock()->now(),
        prev_time = this->get_clock()->now();
        float dt;
        rclcpp::Rate rate(100ms);

        // compute and send linear velocity, 
        // until the robot reaches the waypoint
        while (
            (std::abs(wp(0) - current_pose_(0)) > 0.04 || 
            std::abs(wp(1) - current_pose_(1)) > 0.04) && 
            rclcpp::ok()) {
            waypoint_orientation_ = std::atan2(
            wp(1) - current_pose_(1),
            wp(0) - current_pose_(0));
            err_orientation_ = waypoint_orientation_ - orientation_;
            if (err_orientation_ > 0.05 && (wp - current_pose_).norm() > 0.2) {
                cmd_vel_.linear.x = 0;
                cmd_vel_.linear.y = 0;
                twist_pub_->publish(cmd_vel_);
                turn(wp);
            }
            err_pose = (wp - current_pose_).norm();
            current_time = this->get_clock()->now();
            dt = (current_time - prev_time).seconds();
            sum_I += err_pose * dt;
            X_dot = (current_pose_ - prev_pose).norm()/dt;

            // PID controller (linear velocity)
            input = 
                kP_pose_*err_pose + 
                kI_pose_*sum_I + 
                kD_pose_*X_dot;

            cmd_vel_.linear.x = input;
            twist_pub_->publish(cmd_vel_);

            prev_pose = current_pose_;
            prev_time = current_time;

            std::this_thread::sleep_for(100ms);
        }
       
        cmd_vel_.linear.x = 0;
        cmd_vel_.linear.y = 0;
        twist_pub_->publish(cmd_vel_);
    }

    void turn(const Eigen::Vector2f wp){
        float prev_orientation = orientation_;
        float sum_I = 0.0;
        float X_dot = 0.0;
        float input;

        rclcpp::Time current_time = this->get_clock()->now(),
        prev_time = this->get_clock()->now();
        float dt;
        rclcpp::Rate rate(100ms);

        waypoint_orientation_ = std::atan2(
            wp(1) - current_pose_(1),
            wp(0) - current_pose_(0));
        err_orientation_ = waypoint_orientation_ - orientation_;

        // compute and send angular velocity, 
        // until the robot is align with the waypoint
        while (std::abs(err_orientation_) >= 0.02 && rclcpp::ok()) {
            waypoint_orientation_ = std::atan2(
                wp(1) - current_pose_(1),
                wp(0) - current_pose_(0));
            err_orientation_ = waypoint_orientation_ - orientation_;
            current_time = this->get_clock()->now();
            dt = (current_time - prev_time).seconds();

            sum_I += err_orientation_ * dt;
            X_dot = (orientation_ - prev_orientation)/dt;

            // PID controller (angular velocity)
            input = 
                kP_orientation_*err_orientation_ + 
                kI_orientation_*sum_I + 
                kD_orientation_*X_dot;

            if(orientation_*waypoint_orientation_ < -3) {
                input = -input;
            }
            cmd_vel_.angular.z = scene_number_ > 1? - input: input;
            twist_pub_->publish(cmd_vel_);

            prev_orientation= orientation_;
            prev_time = current_time;
            std::this_thread::sleep_for(100ms);
        }
        // stop robot motion when waypoint is reached
        cmd_vel_.angular.z = 0;
        twist_pub_->publish(cmd_vel_);
    }

    std::vector<Eigen::Vector2f> readPointsFromYAML() {
    std::vector<Eigen::Vector2f> setpoints;

    // Get the package's share directory and append the YAML file path
    std::string package_share_directory =
        ament_index_cpp::get_package_share_directory(
            "pid_maze_solver"); // Replace with your package name

    auto waypoint_file_name = "sim_points.yaml";

    switch (scene_number_) {
    case 1: // Simulation
      waypoint_file_name = "sim_points.yaml";
      break;

    case 2: // CyberWorld
      waypoint_file_name = "cyberworld_points.yaml";
      break;

    default:
      RCLCPP_ERROR(this->get_logger(), "Invalid scene number: %d",
                   scene_number_);
      return setpoints;
    }

    RCLCPP_INFO(this->get_logger(), "############# Waypoint file loaded: %s",
                waypoint_file_name);

    std::string yaml_file =
        package_share_directory + "/waypoints/" + waypoint_file_name;

    // Read points from the YAML file
    try {
        YAML::Node config = YAML::LoadFile(yaml_file);

        if (config["points"]) {
            for(auto p : config["points"]) {
                setpoints.push_back({p[0].as<float>(), p[1].as<float>()});
            }
        }
    } catch (const YAML::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read YAML file: %s",
                   e.what());
    }

    return setpoints;
  }

    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::SubscriptionOptions odom_sub_options_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    geometry_msgs::msg::Twist cmd_vel_;

    Eigen::Vector2f current_pose_;
    float orientation_, err_orientation_, waypoint_orientation_;
    std::vector<Eigen::Vector2f> waypoints_;

    float kP_pose_, kI_pose_, kD_pose_;
    float kP_orientation_, kI_orientation_, kD_orientation_;

    int scene_number_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    int scene_number = 1; // Default scene number
    if (argc > 1) {
        scene_number = std::atoi(argv[1]);
    }
    auto pid_maze_controller = std::make_shared<PIDMazeSolver>(scene_number);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(pid_maze_controller);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}