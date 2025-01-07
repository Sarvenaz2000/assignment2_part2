#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class RobotMovementNode : public rclcpp::Node
{
public:
    RobotMovementNode() : Node("robot_movement_node")
    {
        // Create a publisher for the /cmd_vel topic
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Create a timer to periodically move the robot
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&RobotMovementNode::move_robot, this));
    }

private:
    void move_robot()
    {
        // Create a Twist message to control linear and angular velocities
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.2;  // Forward velocity (m/s)
        msg.angular.z = 0.5; // Angular velocity (rad/s)

        // Publish the message to control the robot's movement
        publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotMovementNode>());
    rclcpp::shutdown();
    return 0;
}

