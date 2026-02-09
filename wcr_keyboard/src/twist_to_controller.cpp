#include <cmath>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace std::chrono_literals;

class TwistToControllerNode : public rclcpp::Node
{
    public:
        TwistToControllerNode() : Node("twist_to_controller") {

            cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                    publish_control(*msg);
                });
            wheel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "/wcr_robot/driving_velocity_controller/commands", 10
            );
            steer_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "/wcr_robot/steering_position_controller/commands", 10
            );
        }
    
    private:
        void publish_control(const geometry_msgs::msg::Twist & cmd) 
        {
            std::vector<double> wheel_velocities(4, cmd.linear.x * 10);
            std::vector<double> steering_angles = {cmd.angular.z / 3, cmd.angular.z / 3, -cmd.angular.z / 3, -cmd.angular.z / 3};
            std_msgs::msg::Float64MultiArray wheel_msg;
            std_msgs::msg::Float64MultiArray steer_msg;

            wheel_msg.data = wheel_velocities;
            steer_msg.data = steering_angles;

            wheel_pub_->publish(wheel_msg);
            steer_pub_->publish(steer_msg);
            
        }

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steer_pub_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TwistToControllerNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}