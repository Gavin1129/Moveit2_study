#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"

class TargetPosePublisher:public rclcpp::Node{
    public:
        TargetPosePublisher():Node("Franka_Commd_Publish_node_cpp"){
            RCLCPP_INFO(this->get_logger(),"Command publisher created!");
            publisher_ = create_publisher<geometry_msgs::msg::Pose>("target_pose", 10);
            timer_ = create_wall_timer(std::chrono::milliseconds(1000),
                               std::bind(&TargetPosePublisher::publishTargetPose, this));
        }
    private:
        void publishTargetPose()
        {
            // Publish the joint state message
            auto target_pose_msg = geometry_msgs::msg::Pose();
            target_pose_msg.position.x = 0.28;
            target_pose_msg.position.y = -0.2;
            target_pose_msg.position.z = 0.5;
            publisher_->publish(target_pose_msg);

        };

        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto franka_listen_node = std::make_shared<TargetPosePublisher>();
    rclcpp::spin(franka_listen_node);
    rclcpp::shutdown();
    return 0;
}