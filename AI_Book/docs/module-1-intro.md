# Module 1: Introduction to ROS2

The Robot Operating System 2 (ROS2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.

## Why ROS2?

ROS2 addresses many limitations of its predecessor, ROS1, especially in areas like:

*   **Real-time Control:** Improved support for real-time operating systems.
*   **Multi-robot Systems:** Better handling of distributed and multi-robot setups.
*   **Security:** Enhanced security features for robust deployments.
*   **New Communication Mechanisms:** DDS (Data Distribution Service) as the underlying communication layer.
*   **Cross-platform Support:** Broader support for various operating systems.

## Key Concepts of ROS2

Understanding these core concepts is crucial for working with ROS2:

### 1. Nodes

Nodes are executable processes that perform computation. They are designed to be modular, with each node being responsible for a single purpose (e.g., a node for reading camera data, a node for controlling motors).

### 2. Topics

Topics are named buses over which nodes exchange messages. A node can publish messages to a topic, or subscribe to a topic to receive messages. This publish/subscribe model enables loosely coupled communication between nodes.

### 3. Messages

Messages are data structures used for communication between nodes via topics. ROS2 provides a wide range of standard message types, and you can also define custom message types.

### 4. Services

Services provide a request/reply communication model. A client node sends a request to a service server, and the server processes the request and returns a response. This is useful for synchronous operations that require a direct response.

### 5. Actions

Actions are used for long-running tasks that provide periodic feedback and can be preempted. They consist of a goal, feedback, and result. This is ideal for tasks like "move to a specific location" where you want to monitor progress and potentially cancel the operation.

### 6. Parameters

Parameters allow you to configure nodes at runtime. Nodes can expose parameters that can be set or changed externally, providing flexibility without recompiling code.

## Installation

This section provides a general overview of installing ROS2. For detailed, up-to-date instructions, always refer to the official ROS2 documentation for your specific operating system and ROS2 distribution (e.g., Foxy, Galactic, Humble).

### Prerequisites (Ubuntu 22.04 LTS as an example)

1.  **Set up locales:**
    ```bash
    locale  # check for UTF-8

    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    locale  # verify settings
    ```

2.  **Add the ROS2 apt repository:**
    ```bash
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```

### Install ROS2 Packages

After setting up the repository, you can install a full ROS2 desktop environment:

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop -y # Replace 'humble' with your desired ROS2 distribution
```

### Environment Setup

To use ROS2, you need to source its setup files in your terminal:

```bash
source /opt/ros/humble/setup.bash # Replace 'humble' with your distribution
```

To automatically source it for every new terminal:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc # For bash shell
echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc # For zsh shell
```

## Basic Usage

### Creating a Workspace

A ROS2 workspace is a directory where you can store and build your packages.

```bash
mkdir -p ros2_ws/src
cd ros2_ws
colcon build
```

### Creating a Package

A package is the basic unit of ROS2 software. It contains nodes, launch files, configuration files, and more.

```bash
cd src
ros2 pkg create --build-type ament_python my_robot_controller # Example for a Python package
# or
ros2 pkg create --build-type ament_cmake my_cpp_controller # Example for a C++ package
```

### Writing a Simple Publisher Node (Python Example)

Let's create a simple Python node that publishes "Hello ROS2!" messages to a topic.

1.  **Create a file** `my_robot_controller/my_robot_controller/publisher_member_function.py` (assuming `my_robot_controller` is your package name):

    ```python
    import rclpy
    from rclpy.node import Node

    from std_msgs.msg import String

    class MinimalPublisher(Node):

        def __init__(self):
            super().__init__('minimal_publisher')
            self.publisher_ = self.create_publisher(String, 'topic', 10)
            timer_period = 0.5  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.i = 0

        def timer_callback(self):
            msg = String()
            msg.data = 'Hello ROS2! %d' % self.i
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.i += 1


    def main(args=None):
        rclpy.init(args=args)

        minimal_publisher = MinimalPublisher()

        rclpy.spin(minimal_publisher)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_publisher.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()
    ```

2.  **Update `setup.py`** in your package directory (`my_robot_controller/setup.py`) to include the executable:

    ```python
    from setuptools import find_packages, setup

    package_name = 'my_robot_controller'

    setup(
        name=package_name,
        version='0.0.0',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='your_name',
        maintainer_email='your_email@example.com',
        description='TODO: Package description',
        license='TODO: License declaration',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'talker = my_robot_controller.publisher_member_function:main', # Add this line
            ],
        },
    )
    ```

3.  **Build your workspace:**

    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_robot_controller
    source install/setup.bash
    ```

4.  **Run the publisher node:**

    ```bash
    ros2 run my_robot_controller talker
    ```

### Writing a Simple Subscriber Node (Python Example)

Now, let's create a simple Python node that subscribes to the topic and prints the received messages.

1.  **Create a file** `my_robot_controller/my_robot_controller/subscriber_member_function.py`:

    ```python
    import rclpy
    from rclpy.node import Node

    from std_msgs.msg import String


    class MinimalSubscriber(Node):

        def __init__(self):
            super().__init__('minimal_subscriber')
            self.subscription = self.create_subscription(
                String,
                'topic',
                self.listener_callback,
                10)
            self.subscription  # prevent unused variable warning

        def listener_callback(self, msg):
            self.get_logger().info('I heard: "%s"' % msg.data)


    def main(args=None):
        rclpy.init(args=args)

        minimal_subscriber = MinimalSubscriber()

        rclpy.spin(minimal_subscriber)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()
    ```

2.  **Update `setup.py`** again to include the new executable:

    ```python
    from setuptools import find_packages, setup

    package_name = 'my_robot_controller'

    setup(
        name=package_name,
        version='0.0.0',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='your_name',
        maintainer_email='your_email@example.com',
        description='TODO: Package description',
        license='TODO: License declaration',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'talker = my_robot_controller.publisher_member_function:main',
                'listener = my_robot_controller.subscriber_member_function:main', # Add this line
            ],
        },
    )
    ```

3.  **Build your workspace:**

    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_robot_controller
    source install/setup.bash
    ```

4.  **Run the subscriber node (in a new terminal, after running the talker):**

    ```bash
    ros2 run my_robot_controller listener
    ```

    You should see the "I heard: 'Hello ROS2! %d'" messages being printed in the subscriber terminal.

## Further Exploration

*   **ROS2 CLI Tools:** Explore commands like `ros2 topic list`, `ros2 node list`, `ros2 param get`, etc.
*   **Launch Files:** Learn how to use launch files (`.launch.py` or `.launch.xml`) to start multiple nodes simultaneously.
*   **Rviz2:** A 3D visualization tool for ROS2.
*   **Gazebo/Ignition:** Physics simulators for robotics.

This module provides a foundational understanding of ROS2. As you progress, you will delve deeper into specific functionalities and advanced applications.
