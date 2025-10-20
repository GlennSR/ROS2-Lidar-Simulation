#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

int main(int argc, char ** argv)
{
  // Move robot forward for 1 second
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("robot_control");
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  geometry_msgs::msg::Twist msg;
  msg.linear.x = 0.5;
  msg.angular.z = 0.0;
  publisher->publish(msg);
  rclcpp::spin_some(node);
  rclcpp::shutdown();
  
  return 0;
}

/*int main(int argc, char ** argv)
{
  // Move robot forward for 1 second
  class RobotController : public rclcpp::Node
  {
    public:
        RobotController() : Node("robot_controller")
        {
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            timer_ = this->create_wall_timer(
                100ms, std::bind(&RobotController::timer_callback, this)
            )
        }
    private:

        
  };

  return 0;
}*/