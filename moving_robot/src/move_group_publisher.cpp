#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

class PositionPublisher : public rclcpp::Node
{
public:
  PositionPublisher() : Node("position_publisher")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Point>("target_position", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000), std::bind(&PositionPublisher::publishPosition, this));
  }

private:
  void publishPosition()
  {
    auto message = geometry_msgs::msg::Point();
    message.x = -0.093;  // Example x-coordinate
    message.y = 0.446;  // Example y-coordinate
    message.z = 0.504;  // Example z-coordinate
    RCLCPP_INFO(this->get_logger(), "Publishing position: x = %f, y = %f, z = %f", message.x, message.y, message.z);
    publisher_->publish(message);
  }

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionPublisher>());
  rclcpp::shutdown();
  return 0;
}
