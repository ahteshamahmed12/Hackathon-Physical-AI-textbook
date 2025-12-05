# Module 2: Gazebo

Gazebo is a powerful 3D robotics simulator that allows developers to accurately and efficiently test algorithms, design robots, and perform regression testing in a safe, virtual environment. It offers robust physics engines, high-quality graphics, and convenient programmatic interfaces.

## 2.1 Introduction to Gazebo

### What is Gazebo?
Gazebo is an open-source 3D simulator for complex robotics scenarios. It provides the ability to simulate populations of robots, sensors, and objects in a 3D environment. Gazebo aims to accurately simulate robots in indoor and outdoor environments, offering a realistic platform for robot development without the need for physical hardware.

### Why use Gazebo for robotics?
*   **Realistic Simulation:** Gazebo incorporates multiple physics engines (e.g., ODE, Bullet, DART, Simbody) to provide realistic rigid-body dynamics, allowing for accurate simulation of robotic movements and interactions.
*   **Sensor Simulation:** It offers high-fidelity simulation of various sensors, including cameras, depth sensors, LIDAR, IMUs, and more, which are crucial for developing perception algorithms.
*   **Visualization:** Gazebo provides a rich graphical interface to visualize the robots and their environment, enabling developers to monitor simulations effectively.
*   **Integration with ROS/ROS 2:** Seamless integration with the Robot Operating System (ROS) and ROS 2 allows for direct control of simulated robots using the same software stack as real robots.
*   **Cost-Effective Development:** It reduces the need for expensive physical prototypes and testing, accelerating the development cycle and reducing costs.

### Key Features of Gazebo
*   **Multiple Physics Engines:** Choose the engine best suited for your simulation needs.
*   **Advanced 3D Graphics:** Visualize environments and robot interactions with high fidelity.
*   **Extensible Plugin Architecture:** Customize simulations with user-defined plugins for sensors, actuators, and environmental control.
*   **Programmatic Interfaces:** Control and interact with simulations using various APIs.
*   **Large Model Database:** Access a vast online repository of robot models and environments.

## 2.2 Gazebo Versions: Classic vs. Sim

Historically, Gazebo Classic has been the go-to simulator for ROS users. However, with the evolution of ROS 2 and underlying technologies, the development focus has shifted to **Gazebo Sim** (formerly Ignition Gazebo).

### Gazebo Classic (End-of-Life 2025)
Gazebo Classic is the older, more established version. It has been widely used with ROS 1 and early ROS 2 distributions. However, it reached its end-of-life in January 2025 and is no longer actively supported or integrated with newer ROS 2 distributions like Jazzy and beyond.

### Gazebo Sim (Ignition Gazebo) - The future for ROS 2
Gazebo Sim is the modern successor to Gazebo Classic, built on a more modular and extensible architecture using the Ignition Robotics framework. It is the recommended simulator for all new ROS 2 projects. Gazebo Sim offers improved performance, better modularity, and a more robust API.

### Migration Considerations
For projects transitioning from Gazebo Classic to Gazebo Sim, key changes include:
*   **Command Line Tools:** Different commands are used to launch simulations (`gz sim` instead of `gazebo`).
*   **Configuration Files:** Changes in SDF (Simulation Description Format) versions and how worlds and models are defined.
*   **ROS Integration:** The primary bridge for ROS 2 communication is `ros_gz_bridge`, which is part of the `ros_gz` metapackage.

## 2.3 Installation and Setup

### Installing Gazebo Sim (with ROS 2)
The recommended way to install Gazebo Sim for ROS 2 is through the official ROS 2 installation instructions, which often bundle Gazebo Sim packages.

Assuming you have a ROS 2 distribution (e.g., Humble, Iron, Jazzy) installed, you can typically install Gazebo Sim and its ROS 2 integration packages using:

```bash
sudo apt update
sudo apt install ros-<ROS_DISTRO>-ros-gz # Replace <ROS_DISTRO> with your ROS 2 distribution, e.g., humble
```

This command installs the `ros_gz` metapackage, which includes `ros_gz_bridge` and other necessary tools for integration.

### Basic Gazebo Interface Overview
Once installed, you can launch a basic Gazebo Sim world:

```bash
gz sim -v 4 empty.sdf
```
*   `gz sim`: The main command to launch Gazebo Sim.
*   `-v 4`: Sets the verbosity level.
*   `empty.sdf`: Loads an empty world defined by an SDF file.

This will open the Gazebo Sim GUI, where you can observe the simulated environment.

### Launching a Simple World
You can also launch more complex worlds. For example, to launch a differential drive robot in a simple world (assuming the necessary models and world files are available through `ros_gz_sim_demos` or similar packages):

```bash
ros2 launch ros_gz_sim_demos diff_drive.launch.py
```
This command typically uses a ROS 2 launch file to bring up Gazebo Sim with a pre-configured robot.

## 2.4 Robot Description Formats: URDF and SDF

Robots and environments in Gazebo are described using specific file formats.

### Unified Robot Description Format (URDF)
URDF is an XML format used in ROS to describe all aspects of a robot, including its kinematic and dynamic properties, visual appearance, and collision geometry. It is primarily a description format for a single robot.

### Simulation Description Format (SDF)
SDF is an XML format used by Gazebo to describe everything in a simulation environment, including robots, static objects (worlds), sensors, and plugins. SDF is more powerful than URDF as it can describe complex environments and multiple robots.

### URDF to SDF Conversion
Gazebo can often load URDF files directly and convert them internally to SDF for simulation. However, for more advanced Gazebo-specific features (like nested models or specific sensor types), defining your robot directly in SDF or using a tool to convert URDF to a more comprehensive SDF can be beneficial.

## 2.5 ROS 2 and Gazebo Integration

The integration between ROS 2 and Gazebo Sim is facilitated by the `ros_gz` metapackage, especially `ros_gz_bridge`.

### Overview of `ros_gz_bridge`
The `ros_gz_bridge` package provides a bi-directional communication bridge between ROS 2 topics/services and Gazebo's transport system. This allows ROS 2 nodes to send commands to robots in Gazebo and receive sensor data back.

**Example of bridging a topic:**

To bridge a ROS 2 topic to a Gazebo topic:
```bash
ros2 run ros_gz_bridge bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
```
This command bridges the `/cmd_vel` ROS 2 `Twist` message to a Gazebo `Twist` message.

### `gazebo_ros_pkgs`
While `ros_gz` handles the bridging, `gazebo_ros_pkgs` (or similar ROS 2 control packages) provide the necessary plugins and utilities for detailed robot control and sensor data exposure within Gazebo. Key components include:
*   **`gazebo_dev`**: Development files for Gazebo-ROS integration.
*   **`gazebo_msgs`**: ROS 2 messages and service definitions for interacting with Gazebo (e.g., getting model states).
*   **`gazebo_ros`**: C++ utilities and plugins that interface ROS 2 with Gazebo's core functionalities.
*   **`gazebo_plugins`**: A collection of Gazebo plugins that provide various functionalities, such as differential drive controllers, camera sensors, and IMUs, exposing their data through ROS 2 topics.

### Time Synchronization (`use_sim_time`)
For accurate simulation, it's crucial to synchronize time between Gazebo and ROS 2. In ROS 2, this is achieved by setting the `use_sim_time` parameter to `true` for all ROS 2 nodes that interact with the simulator. This tells ROS 2 to use the simulation time provided by Gazebo instead of the system's real time.

This is typically set in a launch file:

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Set use_sim_time for all nodes in the launch file
    use_sim_time = True

    # Example node (your robot controller or other ROS 2 node)
    my_robot_controller_node = Node(
        package='my_robot_pkg',
        executable='my_robot_controller',
        name='my_robot_controller',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        my_robot_controller_node,
        # ... other nodes or Gazebo launch calls
    ])
```

## 2.6 Simulating a Robot (Example)

A common example involves simulating a simple mobile robot and controlling it with ROS 2.

### Creating a simple robot model (Conceptual)
For this module, we will focus on *using* existing robot models or understanding the basic structure. Creating a detailed URDF/SDF model involves defining links, joints, inertias, visuals, and collisions. You can find extensive tutorials online for creating these files.

### Launching the robot in Gazebo Sim
To launch a robot, you typically use a ROS 2 launch file that:
1.  Starts Gazebo Sim with a specific world.
2.  Spawns your robot model into the simulation.
3.  Launches `ros_gz_bridge` to connect ROS 2 topics to Gazebo.
4.  Starts your ROS 2 control nodes.

### Basic interaction (e.g., publishing commands from ROS 2)
Once your robot is launched, you can publish commands from ROS 2 to control it. For a differential drive robot, this usually involves publishing `geometry_msgs/msg/Twist` messages to a `/cmd_vel` topic, which is then bridged to Gazebo.

**Example of publishing a Twist message (from a ROS 2 terminal):**

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}" -1
```
This command would make the robot move forward at 0.5 m/s and turn at 0.2 rad/s.

## 2.7 Advanced Topics (Briefly mention)

### Gazebo Plugins
Gazebo's functionality can be extended through plugins. These are shared libraries loaded at runtime that can control models, sensors, or the entire simulation. ROS 2 control plugins are a common type, allowing ROS 2 messages to directly influence robot physics in Gazebo.

### Sensors and Actuators
Gazebo accurately simulates various sensors (cameras, LiDAR, IMU, contact sensors) and actuators (motors, grippers). Understanding their configuration in SDF and how to access their data via ROS 2 is crucial for advanced robotics development.

### Custom Worlds and Models
You can create entirely custom simulation environments, including terrain, buildings, and interactive objects, using SDF files. Similarly, complex robot models can be designed and imported.
