#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointStateListener: public rclcpp::Node{
  public:
    JointStateListener():Node("franka_listener_node_cpp"){
      RCLCPP_INFO(this->get_logger(),"Listener Created!");
      subscription_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          // Callback function when a joint state message is received
          printJointState(msg);
        });
    }
  private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    void printJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      // Print joint state information
      RCLCPP_INFO(get_logger(), "Received Joint State:");
      for (size_t i = 0; i < msg->name.size(); ++i)
      {
        RCLCPP_INFO(get_logger(), "  %s: %f", msg->name[i].c_str(), msg->position[i]);
      }
    }
};

int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);
  auto franka_listener_node = std::make_shared<JointStateListener>();
  rclcpp::spin(franka_listener_node);
  rclcpp::shutdown();
  return 0;
}