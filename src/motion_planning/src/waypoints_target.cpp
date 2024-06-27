#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>("position_target", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "arm");

    // Set the maximum velocity and accel. scaling factor
    move_group_interface.setMaxVelocityScalingFactor(1.0);
    move_group_interface.setMaxAccelerationScalingFactor(1.0);

    // Define custom waypoints
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose target_pose1;
    target_pose1.position.x = 0.6855;
    target_pose1.position.y = 0.0;
    target_pose1.position.z = 1.0;
    waypoints.push_back(target_pose1);

    geometry_msgs::msg::Pose target_pose4;
    target_pose4.position.x = 0.6855;
    target_pose4.position.y = 0.0;
    target_pose4.position.z = 1.1895;
    waypoints.push_back(target_pose4);

    // Set the custom path
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    // Execute the planned trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    move_group_interface.execute(plan);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}