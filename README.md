**Robot Movement**
this Ros2 package contains simple node that controls the movement of a robot by publishing velocity commands to the /cmd_vel topic he node uses
the geometry_msgs::msg::Twist message type to control both the linear and angular velocities of the robot. This example node can be easily adapted 
to control any robot that subscribes to /cmd_vel.


**Dependencies**
ROS 2 (Foxy or later recommended)
geometry_msgs package (part of ROS 2 by default)


Here’s a `README.md` file for your ROS 2 package:

---

# Robot Movement Node

This ROS 2 package contains a simple node, `RobotMovementNode`, which publishes velocity commands to control a robot's movement. The node sends linear and angular velocities to the `/cmd_vel` topic, allowing the robot to move forward with a slight rotation.

## Features

- Publishes velocity commands to the `/cmd_vel` topic at a fixed interval (100 ms).  
- Enables control of both linear and angular velocities using ROS 2 message types.

## Prerequisites

Before using this package, ensure that:

1. ROS 2 (e.g., Humble, Galactic, or Foxy) is installed on your system.  
   [ROS 2 Installation Guide](https://docs.ros.org/en/rolling/Installation.html)  
2. A simulation environment or robot is configured to listen to the `/cmd_vel` topic. For example:
   - Use a simulator like Gazebo or a hardware robot setup that subscribes to `/cmd_vel`.

## Installation

1. **Clone the repository**:  
   Navigate to your ROS 2 workspace and clone this package into the `src` directory:  
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url> robot_movement_node
   ```

2. **Build the package**:  
   After cloning, build the package using `colcon`:  
   ```bash
   cd ~/ros2_ws
   colcon build
   ```

3. **Source the workspace**:  
   Source your workspace to make the package available:  
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

1. **Launch the ROS 2 core**:  
   Open a terminal and start the ROS 2 core:  
   ```bash
   ros2 launch ros_ign_gazebo ign_gazebo.launch.py
   ```

2. **Run the Node**:  
   In a new terminal, launch the `RobotMovementNode`:  
   ```bash
   ros2 run robot_movement_node robot_movement_node
   ```

3. **Observe the Output**:  
   - The node will publish velocity commands to the `/cmd_vel` topic every 100 ms.
   - If using a simulator like Gazebo, you can observe the robot moving in response to these commands.

## Code Explanation

The `RobotMovementNode` class performs the following tasks:

- **Publisher**: Creates a publisher to the `/cmd_vel` topic, using the `geometry_msgs/msg/Twist` message type.
- **Timer**: Sets up a timer to call the `move_robot` function every 100 ms.
- **Message Publishing**: The `move_robot` function creates and publishes `Twist` messages containing:
  - A forward linear velocity (`linear.x = 0.2 m/s`).
  - A slight angular velocity (`angular.z = 0.5 rad/s`) for rotation.

### Node Structure

```cpp
class RobotMovementNode : public rclcpp::Node
{
public:
    RobotMovementNode(); // Constructor to initialize publisher and timer
private:
    void move_robot(); // Publishes Twist messages
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; // Publisher
    rclcpp::TimerBase::SharedPtr timer_; // Timer for periodic publishing
};
```

## Example Message

The node publishes messages of type `geometry_msgs/msg/Twist` with the following structure:
```yaml
linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5
```

## Customization

- **Modify Velocities**: Change `msg.linear.x` or `msg.angular.z` in the `move_robot` method to adjust the robot's speed or rotation.
- **Adjust Timer Interval**: Update the timer duration in the constructor to alter the frequency of published messages.



---




