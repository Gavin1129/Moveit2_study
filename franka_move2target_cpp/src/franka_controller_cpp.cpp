#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include <moveit_visual_tools/moveit_visual_tools.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group");
geometry_msgs::msg::Pose::SharedPtr target_pose = nullptr;

void print(const geometry_msgs::msg::Pose::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received Pose: x=%.2f, y=%.2f, z=%.2f",
        msg->position.x, msg->position.y, msg->position.z);
    target_pose = msg;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);
  auto subscriber = move_group_node->create_subscription<geometry_msgs::msg::Pose>("target_pose",10,print);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "panda_link0", "move_group_tutorial",
                                                      move_group.getRobotModel());                                                   
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  visual_tools.trigger();

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.position.x = target_pose->position.x;
  target_pose1.position.y = target_pose->position.y;
  target_pose1.position.z = target_pose->position.z;
  move_group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  move_group.execute(my_plan);

  rclcpp::spin(move_group_node);
  rclcpp::shutdown();
  return 0;
}





