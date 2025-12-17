---
sidebar_position: 3
title: Week 4 - Launch Files & Parameters
---

# Week 4: Launch Files & Parameters

**Learning Objectives:**
- Understand launch file architecture and benefits
- Write Python and XML launch files
- Use parameters for runtime configuration
- Apply namespaces and remapping
- Manage multi-node systems efficiently

---

## Why Launch Files?

As your robotic system grows, manually starting each node becomes tedious:

```bash
# Without launch files (painful!)
ros2 run my_pkg camera_node &
ros2 run my_pkg detector_node &
ros2 run my_pkg planner_node &
ros2 run my_pkg controller_node &
```

**Problems:**
- Error-prone (easy to forget a node)
- Hard to configure (passing arguments manually)
- Difficult to replicate (no single source of truth)
- No coordination (nodes start in random order)

**Solution: Launch Files** orchestrate multiple nodes with a single command:

```bash
ros2 launch my_pkg robot_system.launch.py
```

**Benefits:**
- ✅ Single command to start entire system
- ✅ Centralized configuration
- ✅ Reproducible deployments
- ✅ Conditional logic (if/else for different scenarios)
- ✅ Parameter management from YAML files

---

## Launch File Formats: Python vs XML

ROS 2 supports two launch file formats:

### Python Launch Files (Recommended)

**Advantages:**
- Full programming language (loops, conditionals, functions)
- Better IDE support (syntax highlighting, linting)
- Easier debugging
- More flexible and powerful

**Use when:** You need complex logic or dynamic configuration

### XML Launch Files

**Advantages:**
- Simpler syntax for basic use cases
- Familiar to ROS 1 users
- Easier for non-programmers to read

**Use when:** Simple, static configurations with no logic needed

---

## Python Launch Files

### Example 1: Simple Two-Node Launch

Create `~/ros2_ws/src/my_pkg/launch/simple_launch.py`:

```python
#!/usr/bin/env python3
"""
Simple launch file demonstrating basic node launching
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch two nodes: a talker and a listener
    """
    return LaunchDescription([
        # Node 1: Publisher
        Node(
            package='demo_nodes_py',
            executable='talker',
            name='talker_node',
            output='screen',  # Print output to terminal
            parameters=[],
        ),

        # Node 2: Subscriber
        Node(
            package='demo_nodes_py',
            executable='listener',
            name='listener_node',
            output='screen',
        ),
    ])
```

**Running:**
```bash
ros2 launch my_pkg simple_launch.py
```

**Output:**
```
[talker_node]: Publishing: 'Hello World: 0'
[listener_node]: I heard: 'Hello World: 0'
[talker_node]: Publishing: 'Hello World: 1'
[listener_node]: I heard: 'Hello World: 1'
```

---

### Example 2: Launch File with Arguments

Create `~/ros2_ws/src/my_pkg/launch/configurable_launch.py`:

```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file with configurable arguments
    """
    # Declare launch arguments
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10',
        description='Publishing rate in Hz'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Node with arguments
    sensor_node = Node(
        package='my_pkg',
        executable='sensor_node',
        name='sensor',
        parameters=[{
            'publish_rate': LaunchConfiguration('publish_rate'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen',
    )

    return LaunchDescription([
        publish_rate_arg,
        use_sim_time_arg,
        sensor_node,
    ])
```

**Running with custom arguments:**
```bash
# Default values
ros2 launch my_pkg configurable_launch.py

# Custom values
ros2 launch my_pkg configurable_launch.py publish_rate:=20 use_sim_time:=true
```

---

### Example 3: Conditional Node Launching

```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Conditionally launch nodes based on arguments
    """
    # Declare argument
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Use real camera or simulated sensor'
    )

    # Real camera node (only if use_camera=true)
    camera_node = Node(
        package='camera_driver',
        executable='camera_node',
        name='camera',
        condition=IfCondition(LaunchConfiguration('use_camera')),
        output='screen',
    )

    # Simulated sensor (only if use_camera=false)
    sim_sensor_node = Node(
        package='sim_sensors',
        executable='fake_camera',
        name='camera',
        condition=UnlessCondition(LaunchConfiguration('use_camera')),
        output='screen',
    )

    return LaunchDescription([
        use_camera_arg,
        camera_node,
        sim_sensor_node,
    ])
```

**Usage:**
```bash
# Use real camera
ros2 launch my_pkg conditional_launch.py use_camera:=true

# Use simulated camera
ros2 launch my_pkg conditional_launch.py use_camera:=false
```

---

## XML Launch Files

### Example: XML Equivalent of Simple Launch

Create `~/ros2_ws/src/my_pkg/launch/simple_launch.xml`:

```xml
<launch>
  <!-- Launch talker node -->
  <node pkg="demo_nodes_py" exec="talker" name="talker_node" output="screen"/>

  <!-- Launch listener node -->
  <node pkg="demo_nodes_py" exec="listener" name="listener_node" output="screen"/>
</launch>
```

### Comparison: Python vs XML

| Feature | Python | XML |
|---------|--------|-----|
| **Conditionals** | ✅ Full if/else | ⚠️ Limited (IfCondition) |
| **Loops** | ✅ for/while loops | ❌ Not supported |
| **Functions** | ✅ Define reusable functions | ❌ Not supported |
| **Debugging** | ✅ Python debugger | ⚠️ Limited |
| **Readability** | ✅ IDE support | ✅ Simple syntax |
| **Recommended For** | Complex systems | Simple configurations |

---

## Parameters: Runtime Configuration

### What are Parameters?

**Parameters** are runtime-configurable values that nodes can read and update:

```
Node Configuration
├── Fixed: Compiled into code (requires rebuild to change)
└── Parameters: Set at runtime (flexible, no rebuild needed)
```

**Examples:**
- Camera frame rate
- PID controller gains
- Sensor topic names
- Debug logging levels

### Declaring Parameters in a Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')

        # Declare parameters with default values
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('robot_name', 'robot_1')
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('debug_mode', False)

        # Get parameter values
        self.update_rate = self.get_parameter('update_rate').value
        self.robot_name = self.get_parameter('robot_name').value
        self.max_speed = self.get_parameter('max_speed').value
        self.debug_mode = self.get_parameter('debug_mode').value

        # Use parameters
        self.get_logger().info(f'Robot: {self.robot_name}')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz')
        self.get_logger().info(f'Max speed: {self.max_speed} m/s')

        # Create timer based on parameter
        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.debug_mode:
            self.get_logger().info('Timer callback executed')

def main(args=None):
    rclpy.init(args=args)
    node = ConfigurableNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Setting Parameters via Launch File

```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_pkg',
            executable='configurable_node',
            name='my_node',
            parameters=[{
                'update_rate': 20.0,
                'robot_name': 'atlas',
                'max_speed': 2.0,
                'debug_mode': True,
            }],
            output='screen',
        ),
    ])
```

### Setting Parameters via Command Line

```bash
# Run node with custom parameters
ros2 run my_pkg configurable_node --ros-args \
  -p update_rate:=15.0 \
  -p robot_name:=optimus \
  -p max_speed:=1.8 \
  -p debug_mode:=true
```

---

## YAML Configuration Files

For complex systems with many parameters, use **YAML files**:

### Example: Parameter YAML File

Create `~/ros2_ws/src/my_pkg/config/robot_params.yaml`:

```yaml
# Robot configuration parameters
configurable_node:
  ros__parameters:
    update_rate: 25.0
    robot_name: "turtlebot3"
    max_speed: 1.2
    debug_mode: false

    # Nested parameters
    sensor_config:
      camera_fps: 30
      lidar_range: 10.0

    # Array parameters
    joint_names: ["joint_1", "joint_2", "joint_3"]
    pid_gains: [1.0, 0.1, 0.05]
```

### Loading YAML in Launch File

```python
#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get path to YAML file
    config_file = os.path.join(
        get_package_share_directory('my_pkg'),
        'config',
        'robot_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='my_pkg',
            executable='configurable_node',
            name='my_node',
            parameters=[config_file],  # Load from YAML
            output='screen',
        ),
    ])
```

### Inspecting Parameters (CLI)

```bash
# List all parameters for a node
ros2 param list /my_node

# Get parameter value
ros2 param get /my_node update_rate

# Set parameter at runtime
ros2 param set /my_node update_rate 30.0

# Dump all parameters to YAML
ros2 param dump /my_node
```

---

## Namespaces: Running Multiple Robots

**Problem:** How to run multiple identical robots without topic/node name conflicts?

**Solution:** Use **namespaces** to create isolated communication spaces.

### Without Namespaces (Conflict!)

```python
# Robot 1
Node(package='my_pkg', executable='sensor_node', name='sensor')  # /sensor
# Robot 2
Node(package='my_pkg', executable='sensor_node', name='sensor')  # ERROR: name conflict!
```

### With Namespaces (Isolated)

```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Robot 1 in namespace /robot1
        Node(
            package='my_pkg',
            executable='sensor_node',
            name='sensor',
            namespace='robot1',  # Node: /robot1/sensor, Topics: /robot1/...
            parameters=[{'robot_id': 1}],
            output='screen',
        ),

        # Robot 2 in namespace /robot2
        Node(
            package='my_pkg',
            executable='sensor_node',
            name='sensor',
            namespace='robot2',  # Node: /robot2/sensor, Topics: /robot2/...
            parameters=[{'robot_id': 2}],
            output='screen',
        ),
    ])
```

**Result:**
```
Nodes:
  /robot1/sensor
  /robot2/sensor

Topics:
  /robot1/sensor_data
  /robot2/sensor_data
```

### Multi-Robot Example with Loop

```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = []

    # Launch 5 robots programmatically
    for i in range(1, 6):
        robot_namespace = f'robot_{i}'

        # Sensor node
        sensor_node = Node(
            package='my_pkg',
            executable='sensor_node',
            name='sensor',
            namespace=robot_namespace,
            parameters=[{'robot_id': i}],
        )

        # Controller node
        controller_node = Node(
            package='my_pkg',
            executable='controller_node',
            name='controller',
            namespace=robot_namespace,
            parameters=[{'robot_id': i}],
        )

        nodes.extend([sensor_node, controller_node])

    return LaunchDescription(nodes)
```

---

## Topic Remapping: Connecting Incompatible Nodes

**Problem:** Node A publishes to `/camera/image`, but Node B subscribes to `/image_raw`.

**Solution:** Use **remapping** to redirect topics without modifying code.

### Basic Remapping

```python
Node(
    package='camera_driver',
    executable='camera_node',
    name='camera',
    remappings=[
        ('/camera/image', '/image_raw'),  # Remap /camera/image → /image_raw
    ],
)
```

### Remapping Example: Sensor Fusion

```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Camera node publishes to /camera/image
        Node(
            package='camera_driver',
            executable='camera_node',
            name='camera',
            # No remapping needed
        ),

        # LIDAR node publishes to /lidar/scan
        Node(
            package='lidar_driver',
            executable='lidar_node',
            name='lidar',
            # No remapping needed
        ),

        # Fusion node expects /image and /scan
        Node(
            package='sensor_fusion',
            executable='fusion_node',
            name='fusion',
            remappings=[
                ('/image', '/camera/image'),  # Map /image → /camera/image
                ('/scan', '/lidar/scan'),     # Map /scan → /lidar/scan
            ],
        ),
    ])
```

---

## Complete Example: Multi-Node Robot System

### System Architecture

```
Robot System Launch
├── sensor_node (namespace: /robot)
│   └── publishes: /robot/sensor_data
├── processor_node (namespace: /robot)
│   └── subscribes: /robot/sensor_data
│   └── publishes: /robot/processed_data
└── controller_node (namespace: /robot)
    └── subscribes: /robot/processed_data
    └── publishes: /robot/cmd_vel
```

### Launch File

Create `~/ros2_ws/src/my_pkg/launch/robot_system.launch.py`:

```python
#!/usr/bin/env python3
"""
Complete robot system launch file demonstrating:
- Multiple nodes
- Namespaces
- Parameters from YAML
- Conditional launching
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('my_pkg')

    # YAML config file
    config_file = os.path.join(pkg_dir, 'config', 'robot_params.yaml')

    # Declare arguments
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='robot',
        description='Namespace for all robot nodes'
    )

    use_visualization_arg = DeclareLaunchArgument(
        'use_visualization',
        default_value='true',
        description='Launch RViz for visualization'
    )

    # Get argument values
    robot_namespace = LaunchConfiguration('robot_namespace')
    use_visualization = LaunchConfiguration('use_visualization')

    # Sensor node
    sensor_node = Node(
        package='my_pkg',
        executable='sensor_node',
        name='sensor',
        namespace=robot_namespace,
        parameters=[config_file],
        output='screen',
    )

    # Processor node
    processor_node = Node(
        package='my_pkg',
        executable='processor_node',
        name='processor',
        namespace=robot_namespace,
        parameters=[config_file],
        output='screen',
    )

    # Controller node
    controller_node = Node(
        package='my_pkg',
        executable='controller_node',
        name='controller',
        namespace=robot_namespace,
        parameters=[config_file],
        output='screen',
    )

    # RViz (conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        condition=IfCondition(use_visualization),
        output='screen',
    )

    return LaunchDescription([
        # Arguments
        robot_namespace_arg,
        use_visualization_arg,

        # Nodes
        sensor_node,
        processor_node,
        controller_node,
        rviz_node,
    ])
```

### Running the System

```bash
# Default configuration
ros2 launch my_pkg robot_system.launch.py

# Custom namespace without visualization
ros2 launch my_pkg robot_system.launch.py \
  robot_namespace:=my_robot \
  use_visualization:=false
```

---

## Launch File Best Practices

### 1. Use Descriptive Names
```python
# ❌ Bad
Node(package='pkg', executable='node', name='n1')

# ✅ Good
Node(package='sensor_pkg', executable='camera_driver', name='front_camera')
```

### 2. Always Set output='screen'
```python
# See node output in terminal
Node(..., output='screen')
```

### 3. Organize with Comments
```python
def generate_launch_description():
    # ========== Arguments ==========
    arg1 = DeclareLaunchArgument(...)

    # ========== Sensor Nodes ==========
    camera = Node(...)
    lidar = Node(...)

    # ========== Processing Nodes ==========
    detector = Node(...)

    return LaunchDescription([...])
```

### 4. Use YAML for Complex Configurations
```python
# ❌ Avoid large inline parameter dictionaries
Node(parameters=[{'p1': 1, 'p2': 2, 'p3': 3, ...}])

# ✅ Use YAML file
Node(parameters=[config_file])
```

### 5. Handle Paths Correctly
```python
# ✅ Always use get_package_share_directory
from ament_index_python.packages import get_package_share_directory
config_file = os.path.join(get_package_share_directory('my_pkg'), 'config', 'params.yaml')
```

---

## CLI Commands for Launch Files

```bash
# List available launch files in a package
ros2 launch my_pkg --show-args

# Show arguments for a specific launch file
ros2 launch my_pkg robot_system.launch.py --show-args

# Launch with arguments
ros2 launch my_pkg robot_system.launch.py arg1:=value1 arg2:=value2

# Launch and see full output (debug)
ros2 launch my_pkg robot_system.launch.py --debug
```

---

## Key Takeaways

1. **Launch files orchestrate multi-node systems** - Single command to start entire robot
2. **Python launch files are more powerful** - Use for complex logic and conditionals
3. **Parameters enable runtime configuration** - No code recompilation needed
4. **YAML files organize parameters** - Cleaner than inline dictionaries
5. **Namespaces isolate multi-robot systems** - Avoid name conflicts
6. **Remapping connects incompatible nodes** - Redirect topics without code changes
7. **Arguments make launch files reusable** - Same file for different configurations

---

## Exercises

1. **Basic Launch File**: Create a launch file that starts 3 nodes: a sensor, a processor, and a controller. Pass different parameters to each.

2. **Conditional Logic**: Write a launch file with an argument `use_sim` that launches either a real sensor driver (`use_sim:=false`) or a simulated sensor (`use_sim:=true`).

3. **Multi-Robot System**: Create a launch file that starts 3 identical robots in different namespaces (`/robot1`, `/robot2`, `/robot3`) using a loop.

4. **YAML Configuration**: Create a YAML parameter file for a robot with at least 10 parameters. Load it in a launch file and verify parameters are set correctly using `ros2 param list`.

5. **Topic Remapping**: You have a camera node publishing to `/camera/image` and a detector node subscribing to `/input_image`. Write a launch file that remaps topics so they communicate.

---

## Next: Week 5 - ROS 2 Packages & Best Practices

In Week 5, we'll learn how to:
- Structure ROS 2 packages properly
- Manage dependencies with package.xml
- Build multi-package workspaces with colcon
- Write tests for ROS 2 nodes
- Follow ROS 2 best practices

Continue to [Week 5: ROS 2 Packages & Best Practices](/docs/module-1/week5-ros2-packages)
