# Module 1: ROS2 for Robot Control

The Robot Operating System 2 (ROS2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.

This module focuses on three critical areas: **topics middleware for robot control**, **bridging Python agents to ROS using rclpy**, and **understanding URDF for humanoid robots**.

## 1.1 ROS2 Fundamentals

### Why ROS2?

ROS2 addresses many limitations of its predecessor, ROS1, especially in areas like:

*   **Real-time Control:** Improved support for real-time operating systems.
*   **Multi-robot Systems:** Better handling of distributed and multi-robot setups.
*   **Security:** Enhanced security features for robust deployments.
*   **New Communication Mechanisms:** DDS (Data Distribution Service) as the underlying communication layer.
*   **Cross-platform Support:** Broader support for various operating systems.

### Key Concepts of ROS2

Understanding these core concepts is crucial for working with ROS2:

#### 1. Nodes

Nodes are executable processes that perform computation. They are designed to be modular, with each node being responsible for a single purpose (e.g., a node for reading camera data, a node for controlling motors).

#### 2. Topics

Topics are named buses over which nodes exchange messages. A node can publish messages to a topic, or subscribe to a topic to receive messages. This publish/subscribe model enables loosely coupled communication between nodes.

#### 3. Messages

Messages are data structures used for communication between nodes via topics. ROS2 provides a wide range of standard message types, and you can also define custom message types.

#### 4. Services

Services provide a request/reply communication model. A client node sends a request to a service server, and the server processes the request and returns a response. This is useful for synchronous operations that require a direct response.

#### 5. Actions

Actions are used for long-running tasks that provide periodic feedback and can be preempted. They consist of a goal, feedback, and result. This is ideal for tasks like "move to a specific location" where you want to monitor progress and potentially cancel the operation.

#### 6. Parameters

Parameters allow you to configure nodes at runtime. Nodes can expose parameters that can be set or changed externally, providing flexibility without recompiling code.

### Installation

This section provides a general overview of installing ROS2. For detailed, up-to-date instructions, always refer to the official ROS2 documentation for your specific operating system and ROS2 distribution (e.g., Foxy, Galactic, Humble).

#### Prerequisites (Ubuntu 22.04 LTS as an example)

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

#### Install ROS2 Packages

After setting up the repository, you can install a full ROS2 desktop environment:

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop -y # Replace 'humble' with your desired ROS2 distribution
```

#### Environment Setup

To use ROS2, you need to source its setup files in your terminal:

```bash
source /opt/ros/humble/setup.bash # Replace 'humble' with your distribution
```

To automatically source it for every new terminal:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc # For bash shell
echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc # For zsh shell
```

### Basic Usage

#### Creating a Workspace

A ROS2 workspace is a directory where you can store and build your packages.

```bash
mkdir -p ros2_ws/src
cd ros2_ws
colcon build
```

#### Creating a Package

A package is the basic unit of ROS2 software. It contains nodes, launch files, configuration files, and more.

```bash
cd src
ros2 pkg create --build-type ament_python my_robot_controller # Example for a Python package
# or
ros2 pkg create --build-type ament_cmake my_cpp_controller # Example for a C++ package
```

#### Writing a Simple Publisher Node (Python Example)

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

#### Writing a Simple Subscriber Node (Python Example)

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

### Further Exploration

*   **ROS2 CLI Tools:** Explore commands like `ros2 topic list`, `ros2 node list`, `ros2 param get`, etc.
*   **Launch Files:** Learn how to use launch files (`.launch.py` or `.launch.xml`) to start multiple nodes simultaneously.
*   **Rviz2:** A 3D visualization tool for ROS2.
*   **Gazebo/Ignition:** Physics simulators for robotics.

---

## 1.2 Topics Middleware for Robot Control

ROS2's middleware layer is built on top of DDS (Data Distribution Service), providing robust, real-time communication for robot control applications. Understanding the middleware architecture is crucial for building reliable robotic systems.

### Understanding DDS and QoS

**DDS (Data Distribution Service)** is a middleware protocol that ROS2 uses for node-to-node communication. Unlike ROS1's custom transport layer, DDS provides standardized communication with configurable Quality of Service (QoS) settings.

**Quality of Service (QoS)** policies allow you to tune communication behavior for different use cases:

*   **Reliability:** `RELIABLE` (guaranteed delivery) vs `BEST_EFFORT` (faster but may drop messages)
*   **Durability:** Whether late-joining subscribers receive historical messages
*   **History:** How many past messages to keep
*   **Deadline:** Maximum expected time between messages
*   **Liveliness:** Mechanism to detect when nodes become unavailable

### QoS Profiles for Robot Control

Different robot control scenarios require different QoS settings:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # QoS for robot commands - Reliable delivery is critical
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS for sensor data - Best effort for low latency
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            cmd_qos
        )

        # Subscriber for laser scan data
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            sensor_qos
        )

        # Timer for control loop (50Hz)
        self.timer = self.create_timer(0.02, self.control_loop)

    def laser_callback(self, msg):
        # Process sensor data
        min_distance = min(msg.ranges)
        self.get_logger().info(f'Minimum obstacle distance: {min_distance:.2f}m')

    def control_loop(self):
        # Send velocity commands to robot
        cmd = Twist()
        cmd.linear.x = 0.5  # Move forward at 0.5 m/s
        cmd.angular.z = 0.0  # No rotation
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Middleware Communication Patterns

#### 1. Command/Feedback Pattern

This pattern is essential for robot control where you send commands and receive feedback:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Publish joint position commands
        self.joint_cmd_pub = self.create_publisher(
            Float64,
            '/joint_position_command',
            10
        )

        # Subscribe to joint state feedback
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.target_position = 1.57  # Target: 90 degrees
        self.current_position = 0.0
        self.timer = self.create_timer(0.1, self.control_callback)

    def joint_state_callback(self, msg):
        if len(msg.position) > 0:
            self.current_position = msg.position[0]

    def control_callback(self):
        # Simple proportional controller
        error = self.target_position - self.current_position

        if abs(error) > 0.01:  # 0.01 rad tolerance
            cmd = Float64()
            cmd.data = self.current_position + (error * 0.1)
            self.joint_cmd_pub.publish(cmd)
            self.get_logger().info(
                f'Position: {self.current_position:.3f}, '
                f'Target: {self.target_position:.3f}, '
                f'Error: {error:.3f}'
            )
        else:
            self.get_logger().info('Target reached!')

def main(args=None):
    rclpy.init(args=args)
    node = JointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 2. Service-Based Control Pattern

For synchronous operations like calibration or mode switching:

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from example_interfaces.srv import SetBool

class RobotServiceController(Node):
    def __init__(self):
        super().__init__('robot_service_controller')

        # Create service clients
        self.enable_client = self.create_client(SetBool, '/enable_motors')
        self.calibrate_client = self.create_client(Trigger, '/calibrate')

        # Wait for services to be available
        while not self.enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for enable_motors service...')

        while not self.calibrate_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for calibrate service...')

    def enable_motors(self, enable: bool):
        request = SetBool.Request()
        request.data = enable

        future = self.enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(
                f'Motors {"enabled" if enable else "disabled"}: '
                f'{future.result().message}'
            )
        else:
            self.get_logger().error('Service call failed')

    def calibrate(self):
        request = Trigger.Request()
        future = self.calibrate_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('Calibration successful')
            else:
                self.get_logger().warning(f'Calibration failed: {future.result().message}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotServiceController()

    # Startup sequence
    node.enable_motors(True)
    node.calibrate()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Real-Time Considerations

For real-time robot control, consider:

1. **Message Size:** Keep messages small for low latency
2. **Publishing Rate:** Match your control loop frequency (typically 50-1000Hz)
3. **QoS Settings:** Use `BEST_EFFORT` for high-frequency sensor data
4. **Node Priority:** Use real-time kernel and set node priorities for critical tasks

---

## 1.3 Bridging Python Agents to ROS using rclpy

Modern robotics increasingly relies on AI agents for autonomous decision-making. This section covers how to bridge intelligent Python agents with ROS2 for robot control.

### Agent Architecture Pattern

An agent-based robot controller typically consists of:

1. **Perception Layer:** Processes sensor data
2. **Decision Layer:** AI/ML model for decision-making
3. **Action Layer:** Sends commands to robot actuators

### Simple Autonomous Agent

Here's a basic obstacle-avoidance agent using rclpy:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class ObstacleAvoidanceAgent(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_agent')

        # State variables
        self.scan_data = None
        self.robot_state = 'EXPLORING'  # States: EXPLORING, AVOIDING, STOPPED

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.perception_callback,
            10
        )

        # Decision loop at 10Hz
        self.timer = self.create_timer(0.1, self.decision_loop)

        self.get_logger().info('Obstacle Avoidance Agent initialized')

    def perception_callback(self, msg):
        """Perception Layer: Process sensor data"""
        self.scan_data = np.array(msg.ranges)
        # Filter invalid readings
        self.scan_data = np.where(
            np.isinf(self.scan_data),
            msg.range_max,
            self.scan_data
        )

    def decision_loop(self):
        """Decision Layer: Make autonomous decisions"""
        if self.scan_data is None:
            return

        # Divide scan into sectors
        n = len(self.scan_data)
        front_sector = self.scan_data[n//3:2*n//3]
        left_sector = self.scan_data[:n//3]
        right_sector = self.scan_data[2*n//3:]

        min_front = np.min(front_sector)
        min_left = np.min(left_sector)
        min_right = np.min(right_sector)

        # Decision logic
        cmd = Twist()

        if min_front > 1.0:
            # Clear ahead - move forward
            self.robot_state = 'EXPLORING'
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0

        elif min_front < 0.5:
            # Obstacle close - stop and turn
            self.robot_state = 'AVOIDING'
            cmd.linear.x = 0.0
            # Turn towards more open space
            if min_left > min_right:
                cmd.angular.z = 0.5  # Turn left
            else:
                cmd.angular.z = -0.5  # Turn right

        else:
            # Slow approach
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0

        # Action Layer: Execute decision
        self.execute_action(cmd)

        self.get_logger().info(
            f'State: {self.robot_state}, '
            f'Front: {min_front:.2f}m, '
            f'Left: {min_left:.2f}m, '
            f'Right: {min_right:.2f}m'
        )

    def execute_action(self, cmd: Twist):
        """Action Layer: Send commands to robot"""
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    agent = ObstacleAvoidanceAgent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Integrating AI/ML Models with ROS2

Here's an example of integrating a machine learning model with ROS2:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
# import torch  # Uncomment if using PyTorch
# import tensorflow as tf  # Uncomment if using TensorFlow

class MLAgentController(Node):
    def __init__(self):
        super().__init__('ml_agent_controller')

        self.bridge = CvBridge()

        # Load your trained model here
        # self.model = torch.load('path/to/model.pth')
        # self.model.eval()

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publish control commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('ML Agent Controller ready')

    def image_callback(self, msg):
        """Process image and make decisions"""
        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Preprocess image for model
        processed_image = self.preprocess_image(cv_image)

        # Run inference
        action = self.predict_action(processed_image)

        # Convert model output to ROS command
        cmd = self.action_to_twist(action)

        # Publish command
        self.cmd_pub.publish(cmd)

    def preprocess_image(self, image):
        """Preprocess image for ML model"""
        # Example preprocessing
        resized = cv2.resize(image, (224, 224))
        normalized = resized / 255.0
        return normalized

    def predict_action(self, image):
        """Run ML model inference"""
        # Example: mock prediction
        # In practice, use your trained model:
        # with torch.no_grad():
        #     output = self.model(torch.tensor(image))
        #     action = output.argmax()

        # Mock action: [linear_vel, angular_vel]
        action = np.array([0.3, 0.1])
        return action

    def action_to_twist(self, action):
        """Convert model output to Twist message"""
        cmd = Twist()
        cmd.linear.x = float(action[0])
        cmd.angular.z = float(action[1])
        return cmd

def main(args=None):
    rclpy.init(args=args)
    agent = MLAgentController()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### State Machine Agent Pattern

For complex behaviors, use a state machine:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from enum import Enum
import numpy as np

class RobotState(Enum):
    IDLE = 0
    EXPLORING = 1
    APPROACHING_TARGET = 2
    AVOIDING_OBSTACLE = 3
    TARGET_REACHED = 4

class StateMachineAgent(Node):
    def __init__(self):
        super().__init__('state_machine_agent')

        self.state = RobotState.IDLE
        self.scan_data = None
        self.target_position = [5.0, 5.0]  # Example target
        self.current_position = [0.0, 0.0]

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        self.timer = self.create_timer(0.1, self.state_machine_loop)

        # Transition to exploring state
        self.transition_to(RobotState.EXPLORING)

    def scan_callback(self, msg):
        self.scan_data = np.array(msg.ranges)

    def state_machine_loop(self):
        """Main state machine loop"""
        if self.state == RobotState.IDLE:
            self.handle_idle()
        elif self.state == RobotState.EXPLORING:
            self.handle_exploring()
        elif self.state == RobotState.APPROACHING_TARGET:
            self.handle_approaching()
        elif self.state == RobotState.AVOIDING_OBSTACLE:
            self.handle_avoiding()
        elif self.state == RobotState.TARGET_REACHED:
            self.handle_target_reached()

    def handle_idle(self):
        cmd = Twist()  # Zero velocity
        self.cmd_pub.publish(cmd)

    def handle_exploring(self):
        if self.scan_data is None:
            return

        min_distance = np.min(self.scan_data)

        if min_distance < 0.5:
            self.transition_to(RobotState.AVOIDING_OBSTACLE)
        else:
            cmd = Twist()
            cmd.linear.x = 0.3
            self.cmd_pub.publish(cmd)

    def handle_approaching(self):
        # Navigate towards target
        distance_to_target = np.linalg.norm(
            np.array(self.target_position) - np.array(self.current_position)
        )

        if distance_to_target < 0.2:
            self.transition_to(RobotState.TARGET_REACHED)
        else:
            cmd = Twist()
            cmd.linear.x = 0.4
            self.cmd_pub.publish(cmd)

    def handle_avoiding(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5
        self.cmd_pub.publish(cmd)

        # Check if obstacle cleared
        if self.scan_data is not None and np.min(self.scan_data) > 1.0:
            self.transition_to(RobotState.EXPLORING)

    def handle_target_reached(self):
        self.get_logger().info('Target reached! Stopping.')
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def transition_to(self, new_state: RobotState):
        """Handle state transitions"""
        self.get_logger().info(f'State transition: {self.state.name} -> {new_state.name}')
        self.state = new_state

def main(args=None):
    rclpy.init(args=args)
    agent = StateMachineAgent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Agent Communication Patterns

For multi-agent systems, use topics and services for coordination:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class CoordinatingAgent(Node):
    def __init__(self, agent_id):
        super().__init__(f'agent_{agent_id}')

        self.agent_id = agent_id

        # Broadcast to other agents
        self.broadcast_pub = self.create_publisher(
            String, '/agent_broadcast', 10
        )

        # Listen to other agents
        self.broadcast_sub = self.create_subscription(
            String, '/agent_broadcast', self.broadcast_callback, 10
        )

        self.timer = self.create_timer(1.0, self.broadcast_status)

    def broadcast_status(self):
        """Share status with other agents"""
        status = {
            'agent_id': self.agent_id,
            'position': [1.0, 2.0],
            'state': 'EXPLORING',
            'battery': 85.0
        }
        msg = String()
        msg.data = json.dumps(status)
        self.broadcast_pub.publish(msg)

    def broadcast_callback(self, msg):
        """Receive status from other agents"""
        status = json.loads(msg.data)
        if status['agent_id'] != self.agent_id:
            self.get_logger().info(
                f'Received from Agent {status["agent_id"]}: '
                f'State={status["state"]}, Battery={status["battery"]}%'
            )

def main(args=None):
    rclpy.init(args=args)
    agent = CoordinatingAgent(agent_id=1)
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 1.4 Understanding URDF for Humanoids

URDF (Unified Robot Description Format) is an XML-based format for describing robot kinematics, dynamics, and visual properties. For humanoid robots, URDF is essential for simulation, visualization, and motion planning.

### URDF Structure Fundamentals

A URDF file consists of:

1. **Links:** Rigid bodies (limbs, torso, head)
2. **Joints:** Connections between links (revolute, prismatic, fixed)
3. **Visual:** 3D meshes for visualization
4. **Collision:** Simplified geometry for collision detection
5. **Inertial:** Mass and inertia properties

### Basic URDF Example

Here's a simple two-link arm:

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Shoulder Joint -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

  <!-- Upper Arm Link -->
  <link name="upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Elbow Joint -->
  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="0" effort="5" velocity="1.0"/>
  </joint>

  <!-- Forearm Link -->
  <link name="forearm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0"
               iyy="0.005" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

</robot>
```

### Humanoid-Specific Considerations

Humanoid robots have unique requirements:

1. **Kinematic Chains:**
   - Lower body: pelvis → thighs → shins → feet
   - Upper body: torso → shoulders → arms → hands
   - Head: neck joints for pan/tilt

2. **Joint Types:**
   - **Revolute:** Most joints (hips, knees, elbows, shoulders)
   - **Fixed:** Connecting rigid components
   - **Continuous:** Unlimited rotation (rarely used)

3. **Coordinate Frames:**
   - Follow ROS conventions (x-forward, y-left, z-up)
   - Base frame typically at pelvis/torso center

### Simplified Humanoid URDF Structure

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Pelvis (Base Link) -->
  <link name="pelvis">
    <visual>
      <geometry>
        <box size="0.25 0.15 0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0"
               iyy="0.05" iyz="0.0" izz="0.05"/>
    </inertial>
  </link>

  <!-- Torso Joint -->
  <joint name="torso_joint" type="fixed">
    <parent link="pelvis"/>
    <child link="torso"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- Torso Link -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.3 0.4"/>
      </geometry>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <material name="white">
        <color rgba="0.9 0.9 0.9 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="8.0"/>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.1" iyz="0.0" izz="0.05"/>
    </inertial>
  </link>

  <!-- Left Hip Joint -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="pelvis"/>
    <child link="left_thigh"/>
    <origin xyz="0 0.1 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.0" upper="1.57" effort="50" velocity="2.0"/>
  </joint>

  <!-- Left Thigh -->
  <link name="left_thigh">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0"
               iyy="0.04" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left Knee Joint -->
  <joint name="left_knee_joint" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.5" effort="30" velocity="2.0"/>
  </joint>

  <!-- Left Shin -->
  <link name="left_shin">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0"
               iyy="0.03" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Add right leg, arms, and head similarly... -->

</robot>
```

### Parsing URDF with Python and rclpy

```python
import rclpy
from rclpy.node import Node
from urdf_parser_py.urdf import URDF
import xml.etree.ElementTree as ET

class URDFParserNode(Node):
    def __init__(self):
        super().__init__('urdf_parser_node')

        # Load URDF from file
        urdf_file = 'path/to/your/robot.urdf'
        self.robot = self.parse_urdf(urdf_file)

        self.analyze_robot()

    def parse_urdf(self, urdf_file):
        """Parse URDF file"""
        tree = ET.parse(urdf_file)
        root = tree.getroot()

        robot_name = root.get('name')
        self.get_logger().info(f'Parsing URDF for robot: {robot_name}')

        return root

    def analyze_robot(self):
        """Analyze robot structure"""
        # Count links
        links = self.robot.findall('link')
        self.get_logger().info(f'Total links: {len(links)}')

        # Count joints
        joints = self.robot.findall('joint')
        self.get_logger().info(f'Total joints: {len(joints)}')

        # Analyze joints
        for joint in joints:
            joint_name = joint.get('name')
            joint_type = joint.get('type')
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')

            self.get_logger().info(
                f'Joint: {joint_name} ({joint_type}) - '
                f'{parent} -> {child}'
            )

            # Get joint limits
            if joint_type in ['revolute', 'prismatic']:
                limit = joint.find('limit')
                if limit is not None:
                    lower = float(limit.get('lower'))
                    upper = float(limit.get('upper'))
                    self.get_logger().info(
                        f'  Limits: [{lower:.2f}, {upper:.2f}]'
                    )

def main(args=None):
    rclpy.init(args=args)
    node = URDFParserNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Visualizing URDF in RViz2

To visualize your URDF in RViz2:

1. **Create a launch file** (`display.launch.py`):

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get URDF file path
    urdf_file = os.path.join(
        get_package_share_directory('your_package'),
        'urdf',
        'humanoid.urdf'
    )

    # Read URDF file
    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('your_package'),
                'config',
                'urdf_config.rviz'
            )]
        )
    ])
```

2. **Launch the visualization:**

```bash
ros2 launch your_package display.launch.py
```

### Publishing Joint States with Python

Control your URDF robot programmatically:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')

        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.05, self.publish_joint_states)

        # Joint names for humanoid
        self.joint_names = [
            'left_hip_joint',
            'left_knee_joint',
            'right_hip_joint',
            'right_knee_joint',
            'left_shoulder_joint',
            'left_elbow_joint',
            'right_shoulder_joint',
            'right_elbow_joint'
        ]

        self.time = 0.0

    def publish_joint_states(self):
        """Publish animated joint states"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # Simulate walking motion
        self.time += 0.05

        msg.position = [
            0.3 * math.sin(self.time),      # left_hip
            0.5 * abs(math.sin(self.time)), # left_knee
            -0.3 * math.sin(self.time),     # right_hip
            0.5 * abs(math.sin(self.time)), # right_knee
            0.5,                             # left_shoulder
            0.8,                             # left_elbow
            0.5,                             # right_shoulder
            0.8                              # right_elbow
        ]

        msg.velocity = []
        msg.effort = []

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Best Practices for Humanoid URDF

1. **Proper Mass Distribution:** Ensure realistic mass values for stable simulation
2. **Collision Geometry:** Use simplified shapes for performance
3. **Joint Limits:** Set realistic limits based on actual robot capabilities
4. **Coordinate Frames:** Follow ROS conventions consistently
5. **Modular Design:** Use xacro for parametric and reusable URDF components
6. **Testing:** Always visualize in RViz2 before deploying to real hardware

### Advanced: Using xacro for Modular URDF

xacro allows parametric URDF generation:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">

  <!-- Define parameters -->
  <xacro:property name="thigh_length" value="0.4"/>
  <xacro:property name="shin_length" value="0.4"/>
  <xacro:property name="leg_mass" value="3.0"/>

  <!-- Leg macro -->
  <xacro:macro name="leg" params="prefix reflect">

    <joint name="${prefix}_hip_joint" type="revolute">
      <parent link="pelvis"/>
      <child link="${prefix}_thigh"/>
      <origin xyz="0 ${reflect * 0.1} -0.05" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.0" upper="1.57" effort="50" velocity="2.0"/>
    </joint>

    <link name="${prefix}_thigh">
      <visual>
        <geometry>
          <cylinder length="${thigh_length}" radius="0.06"/>
        </geometry>
        <origin xyz="0 0 ${-thigh_length/2}" rpy="0 0 0"/>
      </visual>
      <inertial>
        <mass value="${leg_mass}"/>
        <origin xyz="0 0 ${-thigh_length/2}" rpy="0 0 0"/>
        <inertia ixx="0.04" ixy="0.0" ixz="0.0"
                 iyy="0.04" iyz="0.0" izz="0.01"/>
      </inertial>
    </link>

    <!-- Add knee joint and shin link similarly... -->

  </xacro:macro>

  <!-- Instantiate legs -->
  <xacro:leg prefix="left" reflect="1"/>
  <xacro:leg prefix="right" reflect="-1"/>

</robot>
```

Convert xacro to URDF:
```bash
ros2 run xacro xacro humanoid.urdf.xacro > humanoid.urdf
```

---

This comprehensive module now covers ROS2 fundamentals, topics middleware for robot control, bridging Python agents to ROS using rclpy, and understanding URDF for humanoid robots. These three areas form the foundation for building advanced robotic systems with ROS2.
