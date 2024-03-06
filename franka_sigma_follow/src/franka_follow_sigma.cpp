#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

const std::string MOVE_GROUP = "panda_arm";

class MoveItFollowTarget : public rclcpp::Node
{
public:
  MoveItFollowTarget() : Node("ex_follow_target"),
                         move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP)
  {
    // init_pose_ = this->move_group_.getCurrentPose();
    this->move_group_.setMaxAccelerationScalingFactor(0.1);
    this->move_group_.setMaxVelocityScalingFactor(0.1);
    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("target_pose",rclcpp::QoS(1), 
                  std::bind(&MoveItFollowTarget::target_pose_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Initialization successful.");
  }

private:
  void target_pose_callback(const geometry_msgs::msg::Pose pose)
  {
    if (pose == previous_target_pose_)
    {
      return;
    }

    std::cout << "Robot Position x: " << pose.position.x << std::endl;
    std::cout << "Robot Position y: " << pose.position.y << std::endl;
    std::cout << "Robot Position z: " << pose.position.z << std::endl;

    RCLCPP_INFO(this->get_logger(), "Target pose has changed. Planning and executing...");

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    if (fraction > 0.9) {
        // Execute the plan
        move_group_.execute(trajectory);
    }

    // this->move_group_.setPoseTarget(pose);
    // //   this->move_group_.move();
    // bool success = (move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // if (success){
    //   move_group_.execute(plan);
    // }else{
    //   RCLCPP_ERROR(this->get_logger(), "Planning failed!");
    // }

    previous_target_pose_ = pose;
  }

  moveit::planning_interface::MoveGroupInterface move_group_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_sub_;
  geometry_msgs::msg::Pose previous_target_pose_;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
};


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