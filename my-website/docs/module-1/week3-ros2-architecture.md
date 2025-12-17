---
sidebar_position: 2
title: Week 3 - ROS 2 Architecture & Nodes
---

# Week 3: ROS 2 Architecture & Core Concepts

**Learning Objectives:**
- Understand the ROS 2 graph architecture
- Learn about nodes, topics, services, and actions
- Create your first ROS 2 Python packages
- Implement publisher and subscriber nodes

---

## ROS 2 Architecture Overview

ROS 2 (Robot Operating System 2) is a **middleware framework** for building robotic applications. Think of it as the nervous system of your robot.

### The ROS 2 Graph

```
┌──────────────────────────────────────────────────────────┐
│                    ROS 2 Graph                           │
│                                                          │
│   ┌─────────┐         Topic         ┌─────────┐        │
│   │  Node   │ ──────────────────>   │  Node   │        │
│   │ Camera  │      /image_raw       │ Vision  │        │
│   └─────────┘                       └─────────┘        │
│                                           │              │
│                                           │ Service      │
│                                           ▼              │
│   ┌─────────┐       Action          ┌─────────┐        │
│   │  Node   │ <──────────────────   │  Node   │        │
│   │ Motor   │    /move_to_pose      │ Planner │        │
│   └─────────┘                       └─────────┘        │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

**Key Components:**
1. **Nodes**: Independent processes that perform computation
2. **Topics**: Named buses for asynchronous message passing
3. **Services**: Synchronous request-response communication
4. **Actions**: Asynchronous tasks with feedback and cancellation

---

## 1. Nodes: The Building Blocks

A **node** is an executable process that performs a specific task:
- Camera node: Publishes images
- Motor controller node: Commands motors
- Planner node: Computes paths

### Why Multiple Nodes?

**Modularity:**
- Each node has a single responsibility
- Easy to test, debug, and replace
- Nodes can run on different machines

**Example System:**
```
Robot System
├── camera_node (publishes /image_raw)
├── detector_node (subscribes to /image_raw, publishes /detections)
├── planner_node (subscribes to /detections, publishes /plan)
└── controller_node (subscribes to /plan, commands motors)
```

### Creating Your First Node (Python)

**Step 1: Set up workspace**
```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create package
ros2 pkg create --build-type ament_python my_first_package \
  --dependencies rclpy std_msgs
```

**Step 2: Write a simple node**

Create `~/ros2_ws/src/my_first_package/my_first_package/hello_node.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class HelloNode(Node):
    def __init__(self):
        # Initialize node with name 'hello_node'
        super().__init__('hello_node')

        # Create a timer that calls timer_callback every 1.0 seconds
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

        self.get_logger().info('Hello Node has started!')

    def timer_callback(self):
        """Called every 1 second"""
        self.get_logger().info(f'Hello ROS 2! Count: {self.counter}')
        self.counter += 1

def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create node
    node = HelloNode()

    # Spin (process callbacks)
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Step 3: Build and run**
```bash
# Build
cd ~/ros2_ws
colcon build --packages-select my_first_package

# Source
source install/setup.bash

# Run
ros2 run my_first_package hello_node
```

**Output:**
```
[INFO] [hello_node]: Hello Node has started!
[INFO] [hello_node]: Hello ROS 2! Count: 0
[INFO] [hello_node]: Hello ROS 2! Count: 1
[INFO] [hello_node]: Hello ROS 2! Count: 2
...
```

---

## 2. Topics: Publish-Subscribe Communication

**Topics** enable many-to-many communication:
- Publishers send messages
- Subscribers receive messages
- Decoupled: publishers/subscribers don't need to know about each other

### Topic Architecture

```
Publisher Node 1 ──┐
                   │
Publisher Node 2 ──┼──> Topic: /sensor_data ──┬──> Subscriber Node 1
                   │                           │
Publisher Node 3 ──┘                           └──> Subscriber Node 2
```

### Publisher Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')

        # Create publisher on topic '/chatter'
        # Queue size = 10 (buffer for messages)
        self.publisher = self.create_publisher(String, '/chatter', 10)

        # Publish every 0.5 seconds
        self.timer = self.create_timer(0.5, self.publish_message)
        self.counter = 0

    def publish_message(self):
        msg = String()
        msg.data = f'Hello, ROS 2! Message #{self.counter}'

        # Publish
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')

        # Create subscription to topic '/chatter'
        self.subscription = self.create_subscription(
            String,
            '/chatter',
            self.listener_callback,
            10  # QoS queue size
        )

    def listener_callback(self, msg):
        """Called when a message is received"""
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Running Publisher & Subscriber

**Terminal 1 (Publisher):**
```bash
ros2 run my_first_package simple_publisher
```

**Terminal 2 (Subscriber):**
```bash
ros2 run my_first_package simple_subscriber
```

**Output (Subscriber):**
```
[INFO] [simple_subscriber]: Received: "Hello, ROS 2! Message #0"
[INFO] [simple_subscriber]: Received: "Hello, ROS 2! Message #1"
[INFO] [simple_subscriber]: Received: "Hello, ROS 2! Message #2"
```

### Inspecting Topics (CLI)

```bash
# List all topics
ros2 topic list

# Show topic info
ros2 topic info /chatter

# Echo topic messages
ros2 topic echo /chatter

# Show message rate
ros2 topic hz /chatter

# Publish from command line
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from CLI'"
```

---

## 3. Services: Request-Response Communication

**Services** enable synchronous communication:
- Client sends request
- Server processes and returns response
- Blocks until response received

### When to Use Services vs Topics?

| Use Case | Communication Type |
|----------|-------------------|
| Sensor data streaming | **Topic** (continuous) |
| Command to turn on LED | **Service** (one-time) |
| Query robot pose | **Service** (request-response) |
| Camera images | **Topic** (high-frequency) |

### Service Example: Add Two Integers

**1. Define service (use built-in `example_interfaces/srv/AddTwoInts`)**

```
# Request
int64 a
int64 b
---
# Response
int64 sum
```

**2. Service Server**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')

        # Create service
        self.service = self.create_service(
            AddTwoInts,
            '/add_two_ints',
            self.handle_add_two_ints
        )

        self.get_logger().info('Add Two Ints Server is ready.')

    def handle_add_two_ints(self, request, response):
        """Service callback: compute sum"""
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**3. Service Client**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')

        # Create client
        self.client = self.create_client(AddTwoInts, '/add_two_ints')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def send_request(self, a, b):
        """Send request to server"""
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Call service (async)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Result: {a} + {b} = {response.sum}')
            return response.sum
        else:
            self.get_logger().error('Service call failed')
            return None

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print('Usage: ros2 run my_first_package add_two_ints_client <a> <b>')
        return

    node = AddTwoIntsClient()
    a = int(sys.argv[1])
    b = int(sys.argv[2])
    node.send_request(a, b)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Running:**
```bash
# Terminal 1: Start server
ros2 run my_first_package add_two_ints_server

# Terminal 2: Call service
ros2 run my_first_package add_two_ints_client 5 7
# Output: Result: 5 + 7 = 12
```

---

## 4. Actions: Long-Running Tasks

**Actions** are for tasks that:
- Take time to complete (seconds to minutes)
- Provide progress feedback
- Can be canceled

**Examples:**
- Navigate to a goal (robot moving)
- Grasp an object (multi-step manipulation)
- Charge battery (long-duration task)

### Action Structure

```
Goal ───────────────────> Action Server
                              │
                              │ Processing...
                              │
Feedback <──────────────────  │
(e.g., "50% complete")        │
                              │
Result  <──────────────────  │
(Final outcome)
```

### Action Example: Fibonacci Sequence

We'll create an action that computes Fibonacci numbers with feedback.

**1. Define action** (simplified - in practice, use `.action` files)

```
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] partial_sequence
```

**2. Action Server** (simplified example)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from action_tutorials_interfaces.action import Fibonacci
import time

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')

        self.action_server = ActionServer(
            self,
            Fibonacci,
            '/fibonacci',
            self.execute_callback
        )

        self.get_logger().info('Fibonacci Action Server started')

    def execute_callback(self, goal_handle):
        """Execute Fibonacci action"""
        self.get_logger().info(f'Executing goal: Fibonacci({goal_handle.request.order})')

        # Initialize feedback
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        # Compute Fibonacci
        for i in range(1, goal_handle.request.order):
            # Check if goal is canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            # Compute next Fibonacci number
            next_num = feedback_msg.partial_sequence[-1] + feedback_msg.partial_sequence[-2]
            feedback_msg.partial_sequence.append(next_num)

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')

            time.sleep(0.5)  # Simulate work

        # Return result
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Testing the action:**
```bash
# Start server
ros2 run my_first_package fibonacci_action_server

# Send goal (CLI)
ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 10}" --feedback
```

---

## Comparison: Topics vs Services vs Actions

| Feature | Topics | Services | Actions |
|---------|--------|----------|---------|
| **Communication** | Many-to-many | One-to-one | One-to-one |
| **Pattern** | Publish-subscribe | Request-response | Goal-feedback-result |
| **Timing** | Asynchronous | Synchronous | Asynchronous |
| **Feedback** | No | No | Yes |
| **Cancelable** | N/A | No | Yes |
| **Use Case** | Sensor data | Quick queries | Long tasks |

---

## Key Takeaways

1. **ROS 2 is a graph-based middleware** connecting independent nodes
2. **Nodes** are modular, single-purpose processes
3. **Topics** enable asynchronous, many-to-many communication (sensor data, commands)
4. **Services** provide synchronous request-response (queries, one-time actions)
5. **Actions** handle long-running tasks with feedback and cancellation
6. **Python interface (rclpy)** makes ROS 2 accessible and beginner-friendly

## Next: Week 4 - Launch Files & Parameters

In Week 4, we'll learn how to:
- Launch multiple nodes simultaneously
- Configure nodes with parameters
- Create complex launch files for entire robot systems

---

## Exercises

1. **Publisher-Subscriber**: Create a temperature sensor publisher and a logger subscriber
2. **Service**: Implement a service that converts Celsius to Fahrenheit
3. **Multi-Node System**: Build a 3-node system: sensor → processor → actuator

