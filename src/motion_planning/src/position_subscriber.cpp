#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>

rclcpp::Node::SharedPtr node = nullptr;

/* We do not recommend this style anymore, because composition of multiple
 * nodes in the same executable is not possible. Please see one of the subclass
 * examples for the "new" recommended styles. This example is only included
 * for completeness because it is similar to "classic" standalone ROS nodes. */

void topic_callback(const geometry_msgs::msg::Pose::SharedPtr msg, moveit::planning_interface::MoveGroupInterface& move_group_interface)
{
    double px = msg->position.x;
    double py = msg->position.y;
    double pz = msg->position.z;
    double ox = msg->orientation.x;
    double oy = msg->orientation.y;
    double oz = msg->orientation.z;
    double ow = msg->orientation.w;

    RCLCPP_INFO(node->get_logger(), "I heard position x: '%.2f'", px);
    RCLCPP_INFO(node->get_logger(), "I heard position y: '%.2f'", py);
    RCLCPP_INFO(node->get_logger(), "I heard position z: '%.2f'", pz);
    RCLCPP_INFO(node->get_logger(), "I heard orientation x: '%.2f'", ox);
    RCLCPP_INFO(node->get_logger(), "I heard orientation y: '%.2f'", oy);
    RCLCPP_INFO(node->get_logger(), "I heard orientation z: '%.2f'", oz);
    RCLCPP_INFO(node->get_logger(), "I heard orientation w: '%.2f'", ow);

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_moveit");

    // Set a target Pose
    auto const target_pose = [&px, &py, &pz, &ox, &oy, &oz, &ow]
    {
        geometry_msgs::msg::Pose target;
        target.position.x = px;
        target.position.y = py;
        target.position.z = pz;
        target.orientation.x = ox;
        target.orientation.y = oy;
        target.orientation.z = oz;
        target.orientation.w = ow;
        return target;
    }();

    move_group_interface.setPoseTarget(target_pose);

    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success)
    {
        move_group_interface.execute(plan);
    }
    else
    {
        RCLCPP_ERROR(logger, "Planing failed!");
    }
}

int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("position_subscriber", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Create the MoveIt MoveGroup Interface
    auto move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm");

    // Set the maximum velocity and accel. scaling factor
    move_group_interface->setMaxVelocityScalingFactor(1.0);
    move_group_interface->setMaxAccelerationScalingFactor(1.0);

    // Subscribe to position topic
    auto subscription = node->create_subscription<geometry_msgs::msg::Pose>("position", 10, [move_group_interface](const geometry_msgs::msg::Pose::SharedPtr msg)
    {
    topic_callback(msg, *move_group_interface);
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}