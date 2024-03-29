#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

const std::string MOVE_GROUP = "panda_arm";

class MoveItFollowTarget : public rclcpp::Node
{
public:
  /// Constructor
  MoveItFollowTarget();

  /// Move group interface for the robot
  moveit::planning_interface::MoveGroupInterface move_group_;
  /// Subscriber for target pose
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_sub_;
  /// Target pose that is used to detect changes
  geometry_msgs::msg::Pose previous_target_pose_;

  moveit::planning_interface::MoveGroupInterface::Plan plan;

private:
  /// Callback that plans and executes trajectory each time the target pose is changed
  void target_pose_callback(const geometry_msgs::msg::Pose pose);
};

MoveItFollowTarget::MoveItFollowTarget() : Node("ex_follow_target"),
                                           move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP)
{
  // Use upper joint velocity and acceleration limits
  this->move_group_.setMaxAccelerationScalingFactor(0.1);
  this->move_group_.setMaxVelocityScalingFactor(0.1);

  // Subscribe to target pose
  target_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("target_pose", rclcpp::QoS(1), std::bind(&MoveItFollowTarget::target_pose_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

void MoveItFollowTarget::target_pose_callback(const geometry_msgs::msg::Pose pose)
{
  // Return if target pose is unchanged

  RCLCPP_INFO(this->get_logger(), "Target pose has changed. Planning and executing...");

  if (pose == previous_target_pose_)
  {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Target pose has changed. Planning and executing...");

  // Plan and execute motion
  this->move_group_.setPoseTarget(pose);
//   this->move_group_.move();
  bool success = (move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success) {
    move_group_.execute(plan);
  } else {
     RCLCPP_ERROR(this->get_logger(), "Planning failed!");
  }

  // Update for next callback
  previous_target_pose_ = pose;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto target_follower = std::make_shared<MoveItFollowTarget>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(target_follower);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}