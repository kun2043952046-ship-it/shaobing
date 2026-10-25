#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class RosHealthNode : public rclcpp::Node
{
public:
  RosHealthNode()
  : Node("test_ros_health")
  {
    health_publisher_ =
      this->create_publisher<std_msgs::msg::String>(
      "/ros_health",
      10);

    timer_ = this->create_wall_timer(
      1s,
      std::bind(&RosHealthNode::publishHealth, this));

    RCLCPP_INFO(
      this->get_logger(),
      "ROS health node started");
  }

private:
  void publishHealth()
  {
    std_msgs::msg::String message;
    message.data = "health";

    health_publisher_->publish(message);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Health message published");
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  //构造节点
  auto node = std::make_shared<RosHealthNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
