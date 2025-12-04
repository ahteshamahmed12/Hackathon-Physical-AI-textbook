---
id: module-1-intro
title: "Module 1: The Robotic Nervous System (ROS 2)"
sidebar_label: "Module 1"
slug: /
---

# The Robotic Nervous System (ROS 2)

ROS 2 (Robot Operating System 2) is an open-source, meta-operating system for robots. It provides a standardized framework for developing robotic applications, offering tools, libraries, and conventions that simplify the process of building complex robotic systems. ROS 2 enables developers to create modular, reusable, and distributed robot software.

### Core Communication Concepts

At the heart of ROS 2 are its communication mechanisms, which allow different parts of a robotic system (known as nodes) to interact seamlessly. The primary communication patterns are:

*   **Nodes:** The fundamental computational units.
*   **Topics:** For asynchronous, many-to-one data streaming.
*   **Services:** For synchronous, request-response interactions.

## ROS 2 Nodes, Topics, and Services

### 1. Nodes

**Definition:** A Node in ROS 2 is an executable process that performs a specific computation. Think of a node as a single, focused program responsible for a particular task within the larger robotic system. Examples include a camera driver, a motor controller, a navigation algorithm, or a human-robot interface.

**Key Characteristics:**

*   **Modularity:** Nodes are designed to be small, independent, and reusable. This promotes a modular architecture, making it easier to develop, debug, and maintain complex systems.
*   **Distribution:** Nodes can run on different machines across a network, communicating seamlessly as if they were local processes. This distributed nature is crucial for scaling robotic systems.

**Conceptual Diagram Description:**

Imagine a robot's software as a collection of interacting processes. Each process, or *node*, has a specific role:

```
+------------------+
|     Camera       |
|     Node         |
+--------+---------+
         |
         |
+--------V---------+
|     Image        |
|    Processing    |
|       Node       |
+--------+---------+
         |
         |
+--------V---------+
|     Motor        |
|    Controller    |
|       Node       |
+------------------+
```

In this simplified view, the Camera Node captures images, the Image Processing Node processes them (e.g., detects objects), and the Motor Controller Node receives commands to move the robot. Each is an independent node.

**rclpy Code Example (Simple Node):**

This example demonstrates a basic ROS 2 node written in Python using `rclpy`. This node simply initializes itself and then 'spins' (keeps running) until it's shut down, logging a message periodically.

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):

    def __init__(self):
        super().__init__('minimal_node')
        self.counter = 0
        self.timer = self.create_timer(1.0, self.timer_callback) # 1 second timer
        self.get_logger().info('MinimalNode has been started!')

    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f'Hello from MinimalNode! Count: {self.counter}')

def main(args=None):
    rclpy.init(args=args)
    minimal_node = MinimalNode()
    rclpy.spin(minimal_node) # Keep node alive
    minimal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**To run this node (after sourcing your ROS 2 environment):**

```bash
# Save the code as minimal_node.py
python3 minimal_node.py
```

### 2. Topics

**Definition:** Topics are the most common way for nodes to asynchronously exchange data. They implement a *publish-subscribe* communication pattern, where nodes publish messages to a named topic, and other nodes subscribe to that topic to receive those messages.

**Key Characteristics:**

*   **Asynchronous:** Publishers send messages without waiting for subscribers to acknowledge receipt.
*   **One-to-Many:** A single publisher can send messages to multiple subscribers, and a single subscriber can receive messages from multiple publishers (though typically it's one-to-many from publisher to subscribers).
*   **Unidirectional:** Data flows in one direction, from publisher to subscriber.
*   **Message Types:** Messages sent over topics have a predefined structure (message type), ensuring data consistency.

**Conceptual Diagram Description:**

Consider a sensor publishing data that multiple consumers need:

```
+------------------+                    +-------------------+
|     Sensor       | -- publishes --> |  /robot/sensor_data |
|     Node         |                    |      (Topic)      |
+------------------+                    +--------+----------+
                                                 |              |
                                                 V              V
+------------------+             +------------------+   +------------------+
|   Logging        | <-- subscribes -- |    Image         |   |    Data          |
|   Node           |                 |   Processing     |   |    Analysis      |
+------------------+             +------------------+   +------------------+
```

Here, the Sensor Node publishes data to the `/robot/sensor_data` topic. Both the Logging Node and the Image Processing Node subscribe to this topic, receiving the same stream of data.

**rclpy Code Example (Publisher and Subscriber):**

First, a publisher node (`simple_publisher.py`):

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Standard ROS 2 String message type

class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'chat_topic', 10) # Topic name: chat_topic, QoS: 10
        self.timer = self.create_timer(0.5, self.timer_callback) # Publish every 0.5 seconds
        self.i = 0
        self.get_logger().info('SimplePublisher has been started!')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2! Message count: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Next, a subscriber node (`simple_subscriber.py`):

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):

    def __init__(self):
        super().__init__('simple_subscriber')
        # Subscribe to the 'chat_topic'. The callback will be called when a message is received.
        self.subscription = self.create_subscription(
            String,
            'chat_topic',
            self.listener_callback,
            10) # QoS: 10
        self.subscription # prevent unused variable warning
        self.get_logger().info('SimpleSubscriber has been started!')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**To run these examples:**

Open two separate terminal windows, source your ROS 2 environment in each, and run:

```bash
# Terminal 1:
python3 simple_publisher.py

# Terminal 2:
python3 simple_subscriber.py
```

### 3. Services

**Definition:** Services in ROS 2 provide a synchronous request/reply communication pattern. Unlike topics, which are streams of data, services are used when a node needs to request a specific action or piece of information from another node and expects an immediate response.

**Key Characteristics:**

*   **Synchronous:** The client node sends a request and blocks until it receives a response from the service server.
*   **One-to-One:** A service interaction involves a single client making a request to a single server.
*   **Bidirectional:** Data flows in both directions: a request from client to server, and a response from server to client.
*   **Service Types:** Like message types for topics, services have predefined request and response structures.

**Conceptual Diagram Description:**

Imagine a robot asking a computational server for a complex calculation:

```
+------------------+                   +------------------+
|     Client       | -- Request -->    |      Service     |
|      Node        | <-- Response --   |       Node       |
+------------------+                   +------------------+
```

Here, the Client Node sends a request (e.g., "add these two numbers") to the Service Node, which processes the request and sends back a response (e.g., "the sum is X"). The client waits for the response before continuing.

**rclpy Code Example (Service Server and Client):**

First, define a custom service interface. Create a file `ros2_ws/src/my_py_pkg/srv/AddTwoInts.srv` with the following content:

```
int64 a
int64 b
---
int64 sum
```

Then, in your `setup.py` (assuming it's within a ROS 2 package, e.g., `my_py_pkg` in `ros2_ws/src/my_py_pkg/`):

Add `'std_msgs', 'example_interfaces'` to `data_files` if not present and ensure your `package.xml` lists `build_type=ament_python` and dependencies for `std_msgs` and `rclpy`.

Make sure your `package.xml` includes:

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

And add the service definition generation to your `setup.py`:

```python
# setup.py (partial view)
from setuptools import setup
import os
from glob import glob

package_name = 'my_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
    # ... other setup parameters ...
    install_requires=['rclpy', 'std_msgs'], # Add std_msgs if needed by custom service
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = my_py_pkg.service_member_function:main',
            'client = my_py_pkg.client_member_function:main',
        ],
    },
)
```

After creating the `.srv` file and updating `setup.py` and `package.xml`, you need to build your workspace:

```bash
cd ros2_ws
colcon build --packages-select my_py_pkg
source install/setup.bash
```

Now, the service server node (`my_py_pkg/service_member_function.py`):

```python
import rclpy
from rclpy.node import Node

from my_py_pkg.srv import AddTwoInts # Import the custom service type

class AddTwoIntsService(Node):

    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('AddTwoInts Service Server has been started.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        self.get_logger().info(f'Sending response: sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    add_two_ints_service = AddTwoIntsService()
    rclpy.spin(add_two_ints_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

And the service client node (`my_py_pkg/client_member_function.py`):

```python
import sys
import rclpy
from rclpy.node import Node
from rclpy.task import Future # Import Future for asynchronous handling

from my_py_pkg.srv import AddTwoInts # Import the custom service type

class AddTwoIntsClient(Node):

    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()
        self.get_logger().info('AddTwoInts Service Client has been started.')

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        # Note: rclpy.spin_until_future_complete(self, self.future) would block here.
        # For a non-blocking client, you'd typically integrate this into a larger loop
        # or use executors. For this simple example, we'll spin until complete.
        return self.future

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print('Usage: python3 client_member_function.py <a> <b>')
        return

    add_two_ints_client = AddTwoIntsClient()

    a = int(sys.argv[1])
    b = int(sys.argv[2])

    future = add_two_ints_client.send_request(a, b)

    # Spin the node until the future is complete
    rclpy.spin_until_future_complete(add_two_ints_client, future)

    if future.result() is not None:
        add_two_ints_client.get_logger().info(
            f'Result of add_two_ints: for {a} + {b} = {future.result().sum}')
    else:
        add_two_ints_client.get_logger().error(f'Service call failed %r' % (future.exception(),))

    add_two_ints_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**To run these examples (after building your package and sourcing the install):**

Open two separate terminal windows, source your ROS 2 environment in each, and run:

```bash
# Terminal 1: Run the service server
ros2 run my_py_pkg service

# Terminal 2: Run the service client with arguments
ros2 run my_py_pkg client 5 3
```

You should see the client sending the request and the server responding with the sum (8).
