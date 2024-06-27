#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>

int main(int argc, char* argv[])
{
// Initialize ROS and create the Node
rclcpp::init(argc, argv);
auto const node = std::make_shared<rclcpp::Node>("position_target", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

// Create a ROS logger
auto const logger = rclcpp::get_logger("hello_moveit");
// Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "arm");

// Set the maximum velocity and accel. scaling factor
move_group_interface.setMaxVelocityScalingFactor(1.0);
move_group_interface.setMaxAccelerationScalingFactor(1.0);

// Set a target Pose
auto const target_pose = []
{
    float x=0.0,y=0.0,z = 0.0;
    geometry_msgs::msg::Pose target;
    std::cin >> x;
    std::cin >> y;
    std::cin >> z;

    target.orientation.w = 1.0;
    target.position.x = x;
    target.position.y = y;
    target.position.z = z;
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

rclcpp::spin(node);
rclcpp::shutdown();

return 0;
}
