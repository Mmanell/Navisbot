#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class TwistConverter : public rclcpp::Node
{
public:
  TwistConverter()
  : Node("twist_to_twist_stamped")
  {
    publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>("/output/cmd_vel", rclcpp::QoS(10));
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/input/cmd_vel",
      rclcpp::QoS(10),
      std::bind(&TwistConverter::callback, this, std::placeholders::_1));
  }

private:
  void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    auto stamped_msg = geometry_msgs::msg::TwistStamped();
    stamped_msg.header.stamp = get_clock()->now();
    stamped_msg.header.frame_id = "base_link";  // match your joystick/config
    stamped_msg.twist = *msg;
    publisher_->publish(stamped_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TwistConverter>();       
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
