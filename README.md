**Sarvenaz Ashoori**
**ID : 6878764**


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

1Once **ROS2**, **Python 3**, and the required ROS2 packages are installed, you can begin setting up the workspace.

### Setup
#### 1. Set up your ROS2 workspace

Create a new workspace (or use an existing one) and navigate to the `src` directory:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

#### 2. Install the `robot_urdf` package
Before cloning this repository, you'll need to acquire the `robot_urdf` package, which is responsible for starting the simulation and spawning the robot at position `(2, 2)` within the environment. Clone the `robot_urdf` repository into your workspace’s `src` folder:
```bash
git clone https://github.com/CarmineD8/robot_urdf.git
```

#### 3. Switch to the `ros2` branch
After cloning the `robot_urdf` package, navigate to its directory:
```bash
cd robot_urdf
```
Switch the Git branch from `main` to the `ros2` branch:
```bash
git checkout ros2
```

#### 4. Clone this repository
Now that the `robot_urdf` package is set up, go back to the `src` folder and clone this repository into your workspace’s `src` folder:
```bash
cd ~/ros2_ws/src
git clone https://github.com/Rubin-unige/assignment2_rt_part2.git
```

#### 5. Build the workspace
Once both repositories are cloned, return to the root of your workspace and build the packages using the following command:
```bash
cd ~/ros2_ws
colcon build
```

#### 6. Add the Workspace to Your ROS Environment

To ensure the workspace is sourced automatically whenever you open a new terminal, add it to your `.bashrc` file:
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 7. Source the workspace
After building and sourcing, manually source the workspace for the first time in your current terminal session:
```bash
source ~/ros2_ws/install/setup.bash
```

## Launching Nodes

#### 1. Run the `robot_urdf` Package

Before launching your node, ensure the simulation environment is running. Start the `robot_urdf` package to load the simulation and spawn the robot at position `(2, 2)` in Gazebo:
```bash
ros2 launch robot_urdf gazebo.launch.py
```
This will initiate the simulation environment and spawn the robot within the Gazebo simulator. Wait for the environment to load properly before continuing.

#### 2. Run the `robot_movement_node` Node

Now, you can run the **Python** version of the `robot_movement_node` node using the following command:
```bash
ros2 run assignment2_rt_part2 move_robot
```
This will start the **Python `robot_movement_node`** node.
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

- ## Summary
The **Robot Movement** ROS 2 package provides a simple node that controls a robot's movement by publishing velocity commands to the `/cmd_vel` topic using the `geometry_msgs::msg::Twist` message type. 
It allows the robot to move forward with a slight rotation by setting linear and angular velocities. 
The node publishes messages at a fixed interval (100 ms), and it can be easily adapted for any robot subscribing to `/cmd_vel`. 
The package requires ROS 2 (Foxy or later) and a simulation or robot setup. It includes a straightforward installation process, 
usage instructions, and code that defines the publishing behavior, allowing for easy customization of movement speeds and intervals.



---




