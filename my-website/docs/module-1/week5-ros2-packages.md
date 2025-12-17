---
sidebar_position: 4
title: Week 5 - ROS 2 Packages & Best Practices
---

# Week 5: ROS 2 Packages & Best Practices

**Learning Objectives:**
- Understand ROS 2 package structure and conventions
- Create Python and C++ packages with proper organization
- Manage dependencies using package.xml
- Build multi-package workspaces with colcon
- Write tests for ROS 2 nodes
- Apply best practices for scalable robotic projects

---

## What is a ROS 2 Package?

A **ROS 2 package** is the fundamental unit of ROS 2 software organization. Think of it as a container for related nodes, libraries, configuration files, and documentation.

### Why Packages?

```
Robotic System
├── sensor_pkg (camera, lidar drivers)
├── perception_pkg (object detection, SLAM)
├── planning_pkg (path planning, decision making)
└── control_pkg (motor controllers, actuators)
```

**Benefits:**
- ✅ **Modularity**: Each package has a single, well-defined purpose
- ✅ **Reusability**: Share packages across projects
- ✅ **Dependency Management**: Explicit dependencies prevent conflicts
- ✅ **Version Control**: Independent versioning per package
- ✅ **Collaboration**: Teams work on different packages simultaneously

---

## Package Anatomy

### Python Package Structure

```
my_robot_pkg/
├── package.xml          # Package metadata and dependencies
├── setup.py             # Python package configuration
├── setup.cfg            # Install configuration
├── my_robot_pkg/        # Python module (same name as package)
│   ├── __init__.py      # Makes it a Python module
│   ├── node1.py         # Node implementation
│   ├── node2.py         # Another node
│   └── utils.py         # Utility functions
├── launch/              # Launch files
│   ├── robot.launch.py
│   └── sim.launch.py
├── config/              # Configuration files (YAML, etc.)
│   ├── params.yaml
│   └── rviz.rviz
├── test/                # Unit and integration tests
│   ├── test_node1.py
│   └── test_integration.py
├── resource/            # Resource marker file
│   └── my_robot_pkg
└── README.md            # Package documentation
```

### C++ Package Structure

```
my_cpp_pkg/
├── package.xml          # Package metadata and dependencies
├── CMakeLists.txt       # Build configuration
├── include/             # Public headers
│   └── my_cpp_pkg/
│       └── my_class.hpp
├── src/                 # Source files
│   ├── node1.cpp
│   └── my_class.cpp
├── launch/              # Launch files
│   └── robot.launch.py
├── config/              # Configuration files
│   └── params.yaml
└── test/                # Tests
    └── test_node1.cpp
```

---

## Creating ROS 2 Packages

### Creating a Python Package

**Step 1: Navigate to workspace src directory**
```bash
cd ~/ros2_ws/src
```

**Step 2: Create package**
```bash
ros2 pkg create --build-type ament_python my_robot_pkg \
  --dependencies rclpy std_msgs geometry_msgs sensor_msgs
```

**What this does:**
- Creates package directory structure
- Generates `package.xml` with specified dependencies
- Creates `setup.py` and `setup.cfg`
- Creates Python module directory
- Adds resource marker file

**Output:**
```
going to create a new package
package name: my_robot_pkg
destination directory: /home/user/ros2_ws/src
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['user <user@todo.todo>']
licenses: ['TODO: License declaration']
build type: ament_python
dependencies: ['rclpy', 'std_msgs', 'geometry_msgs', 'sensor_msgs']
creating folder ./my_robot_pkg
creating ./my_robot_pkg/package.xml
creating source and test folder
creating folder ./my_robot_pkg/my_robot_pkg
creating ./my_robot_pkg/setup.py
creating ./my_robot_pkg/setup.cfg
creating folder ./my_robot_pkg/resource
creating ./my_robot_pkg/resource/my_robot_pkg
creating ./my_robot_pkg/test/test_copyright.py
creating ./my_robot_pkg/test/test_flake8.py
creating ./my_robot_pkg/test/test_pep257.py
```

### Creating a C++ Package

```bash
ros2 pkg create --build-type ament_cmake my_cpp_pkg \
  --dependencies rclcpp std_msgs geometry_msgs sensor_msgs
```

---

## package.xml: Package Metadata

The `package.xml` file defines package metadata, dependencies, and build information.

### Basic package.xml Structure

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <!-- Package name (must match directory name) -->
  <name>my_robot_pkg</name>
  <version>1.0.0</version>
  <description>Mobile robot navigation and control package</description>

  <!-- Maintainer information -->
  <maintainer email="dev@example.com">Jane Developer</maintainer>

  <!-- License (choose one: MIT, Apache-2.0, BSD, GPL, etc.) -->
  <license>Apache-2.0</license>

  <!-- Build tool dependency (required) -->
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Runtime dependencies -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>

  <!-- Test dependencies -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <!-- Export information -->
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Dependency Types

| Dependency Type | When to Use |
|----------------|-------------|
| `<buildtool_depend>` | Tools needed to build (e.g., `ament_python`, `ament_cmake`) |
| `<build_depend>` | Packages needed only at build time |
| `<exec_depend>` | Packages needed only at runtime |
| `<depend>` | Packages needed at both build and runtime (most common) |
| `<test_depend>` | Packages needed only for testing |

### Example: Complete package.xml

```xml
<?xml version="1.0"?>
<package format="3">
  <name>robot_navigation</name>
  <version>2.1.0</version>
  <description>
    Autonomous navigation system for mobile robots using ROS 2.
    Includes path planning, obstacle avoidance, and localization.
  </description>

  <maintainer email="robotics@example.com">Robotics Team</maintainer>
  <license>BSD</license>

  <url type="website">https://example.com/robot_navigation</url>
  <url type="repository">https://github.com/example/robot_navigation</url>
  <url type="bugtracker">https://github.com/example/robot_navigation/issues</url>

  <author email="original@example.com">Original Author</author>

  <buildtool_depend>ament_python</buildtool_depend>

  <!-- ROS 2 core -->
  <depend>rclpy</depend>

  <!-- Message types -->
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>

  <!-- External packages -->
  <depend>tf2_ros</depend>
  <depend>nav2_msgs</depend>

  <!-- Python dependencies (system packages) -->
  <exec_depend>python3-numpy</exec_depend>
  <exec_depend>python3-scipy</exec_depend>

  <!-- Testing -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>
  <test_depend>launch_testing</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

---

## setup.py: Python Package Configuration

The `setup.py` file configures how Python packages are built and installed.

### Basic setup.py

```python
from setuptools import setup

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        # Install package marker
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jane Developer',
    maintainer_email='dev@example.com',
    description='Mobile robot navigation package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Format: 'executable_name = package.module:function'
        ],
    },
)
```

### setup.py with Nodes and Launch Files

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),

        # Install URDF files (if any)
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jane Developer',
    maintainer_email='dev@example.com',
    description='Mobile robot navigation package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Register executable nodes
            'sensor_node = my_robot_pkg.sensor_node:main',
            'planner_node = my_robot_pkg.planner_node:main',
            'controller_node = my_robot_pkg.controller_node:main',
        ],
    },
)
```

### Understanding Entry Points

Entry points make your Python scripts executable via `ros2 run`:

```python
entry_points={
    'console_scripts': [
        'my_node = my_robot_pkg.my_node:main',
        #    ↑              ↑          ↑      ↑
        #    |              |          |      |
        # executable    package    module  function
    ],
}
```

**What happens:**
1. `ros2 run my_robot_pkg my_node` is called
2. ROS 2 looks up the entry point
3. Calls `main()` function in `my_robot_pkg/my_node.py`

---

## Building Packages with colcon

### What is colcon?

**colcon** (collective construction) is the build tool for ROS 2. It replaces `catkin_make` from ROS 1.

### Basic Build Workflow

```bash
# Navigate to workspace root
cd ~/ros2_ws

# Build all packages
colcon build

# Build specific package
colcon build --packages-select my_robot_pkg

# Build with symlink install (faster for Python)
colcon build --symlink-install

# Build with verbose output
colcon build --event-handlers console_direct+
```

### Build Directory Structure

After building:
```
ros2_ws/
├── src/                # Source packages
│   └── my_robot_pkg/
├── build/              # Build artifacts (intermediate files)
│   └── my_robot_pkg/
├── install/            # Installed packages (executables, libraries)
│   ├── my_robot_pkg/
│   ├── setup.bash      # Source this to use packages
│   └── setup.zsh
└── log/                # Build logs
    └── latest_build/
```

### Sourcing the Workspace

After building, you must **source** the workspace:

```bash
# Source the install directory
source ~/ros2_ws/install/setup.bash

# Add to ~/.bashrc for automatic sourcing
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

**What sourcing does:**
- Adds package executables to PATH
- Sets up environment variables
- Enables `ros2 run` and `ros2 launch` to find your packages

---

## Complete Example: Creating a Package from Scratch

Let's create a complete robot sensor package step-by-step.

### Step 1: Create Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python robot_sensors \
  --dependencies rclpy std_msgs sensor_msgs geometry_msgs
```

### Step 2: Create a Sensor Node

Create `~/ros2_ws/src/robot_sensors/robot_sensors/lidar_node.py`:

```python
#!/usr/bin/env python3
"""
LIDAR sensor node that publishes laser scan data
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')

        # Declare parameters
        self.declare_parameter('scan_rate', 10.0)
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('num_readings', 360)

        # Get parameters
        self.scan_rate = self.get_parameter('scan_rate').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.num_readings = self.get_parameter('num_readings').value

        # Publisher
        self.publisher = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )

        # Timer
        timer_period = 1.0 / self.scan_rate
        self.timer = self.create_timer(timer_period, self.publish_scan)

        self.get_logger().info(f'LIDAR node started at {self.scan_rate} Hz')

    def publish_scan(self):
        """Generate and publish simulated laser scan"""
        scan = LaserScan()

        # Header
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'

        # Scan parameters
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = (self.angle_max - self.angle_min) / self.num_readings
        scan.time_increment = (1.0 / self.scan_rate) / self.num_readings
        scan.scan_time = 1.0 / self.scan_rate
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        # Simulated ranges (in real node, read from hardware)
        scan.ranges = self.simulate_ranges()
        scan.intensities = []  # Optional intensity data

        self.publisher.publish(scan)

    def simulate_ranges(self):
        """Simulate LIDAR readings (replace with real sensor data)"""
        # Simple simulation: random distances with some obstacles
        ranges = np.random.uniform(
            self.range_min + 1.0,
            self.range_max - 1.0,
            self.num_readings
        )

        # Add some "obstacles" at specific angles
        obstacle_angle_1 = int(self.num_readings * 0.25)  # 90 degrees
        obstacle_angle_2 = int(self.num_readings * 0.75)  # 270 degrees

        ranges[obstacle_angle_1-10:obstacle_angle_1+10] = 2.0  # Obstacle at 2m
        ranges[obstacle_angle_2-10:obstacle_angle_2+10] = 1.5  # Obstacle at 1.5m

        return ranges.tolist()

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Update setup.py

Edit `~/ros2_ws/src/robot_sensors/setup.py`:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'robot_sensors'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Robot sensor drivers and processors',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_node = robot_sensors.lidar_node:main',
        ],
    },
)
```

### Step 4: Create Configuration File

Create `~/ros2_ws/src/robot_sensors/config/lidar_params.yaml`:

```yaml
lidar_node:
  ros__parameters:
    scan_rate: 20.0
    range_min: 0.1
    range_max: 12.0
    angle_min: -3.14159
    angle_max: 3.14159
    num_readings: 720
```

### Step 5: Create Launch File

Create `~/ros2_ws/src/robot_sensors/launch/sensors.launch.py`:

```python
#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get config file path
    config_file = os.path.join(
        get_package_share_directory('robot_sensors'),
        'config',
        'lidar_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot_sensors',
            executable='lidar_node',
            name='lidar',
            parameters=[config_file],
            output='screen',
        ),
    ])
```

### Step 6: Build and Run

```bash
# Build package
cd ~/ros2_ws
colcon build --packages-select robot_sensors --symlink-install

# Source workspace
source install/setup.bash

# Run node directly
ros2 run robot_sensors lidar_node

# Or use launch file
ros2 launch robot_sensors sensors.launch.py
```

### Step 7: Verify It Works

```bash
# Terminal 1: Launch node
ros2 launch robot_sensors sensors.launch.py

# Terminal 2: Check topics
ros2 topic list
# Should show: /scan

# Terminal 3: Echo scan data
ros2 topic echo /scan
```

---

## Testing ROS 2 Packages

### Why Test?

- ✅ Catch bugs early
- ✅ Ensure code quality
- ✅ Enable safe refactoring
- ✅ Document expected behavior
- ✅ Build confidence in deployments

### Types of Tests

1. **Unit Tests**: Test individual functions/classes in isolation
2. **Integration Tests**: Test node communication and system behavior
3. **Launch Tests**: Test launch file configurations

### Unit Test Example

Create `~/ros2_ws/src/robot_sensors/test/test_lidar_node.py`:

```python
#!/usr/bin/env python3
"""
Unit tests for lidar_node
"""
import pytest
import rclpy
from robot_sensors.lidar_node import LidarNode

@pytest.fixture
def node():
    """Fixture to create and cleanup node"""
    rclpy.init()
    test_node = LidarNode()
    yield test_node
    test_node.destroy_node()
    rclpy.shutdown()

def test_node_initialization(node):
    """Test that node initializes correctly"""
    assert node.get_name() == 'lidar_node'
    assert node.scan_rate > 0

def test_simulate_ranges(node):
    """Test range simulation function"""
    ranges = node.simulate_ranges()

    # Check correct number of readings
    assert len(ranges) == node.num_readings

    # Check ranges are within limits
    for r in ranges:
        assert node.range_min <= r <= node.range_max

def test_parameters(node):
    """Test parameter handling"""
    # Get parameters
    scan_rate = node.get_parameter('scan_rate').value
    range_max = node.get_parameter('range_max').value

    assert isinstance(scan_rate, float)
    assert isinstance(range_max, float)
    assert range_max > 0
```

### Running Tests

```bash
# Run all tests in workspace
cd ~/ros2_ws
colcon test

# Run tests for specific package
colcon test --packages-select robot_sensors

# View test results
colcon test-result --all
colcon test-result --verbose
```

---

## ament_python vs ament_cmake

### Feature Comparison

| Feature | ament_python | ament_cmake |
|---------|--------------|-------------|
| **Language** | Python | C++ (or mixed) |
| **Build File** | setup.py | CMakeLists.txt |
| **Speed** | Slower execution | Faster execution |
| **Development** | Fast iteration (symlink) | Requires rebuild |
| **Dependencies** | pip packages | System libraries |
| **Best For** | Rapid prototyping, scripts | Performance-critical, drivers |
| **Learning Curve** | Easier | Steeper |

### When to Use Each

**Use ament_python when:**
- Prototyping and experimentation
- High-level logic and coordination
- Integration with Python ML libraries (PyTorch, TensorFlow)
- Quick iteration is more important than performance

**Use ament_cmake when:**
- Real-time control loops
- Hardware drivers (cameras, motors, sensors)
- Computationally intensive tasks
- Need maximum performance
- Using existing C++ libraries

---

## Best Practices for ROS 2 Packages

### 1. Package Naming

```bash
# ✅ Good: Descriptive, lowercase, underscores
robot_navigation
sensor_drivers
object_detection

# ❌ Bad: Vague, mixed case, hyphens
pkg1
RobotStuff
my-package  # Hyphens not allowed in Python
```

### 2. Single Responsibility Principle

```
# ✅ Good: Each package has one clear purpose
sensor_drivers/     # Only sensor interfaces
perception/         # Only perception algorithms
control/            # Only control logic

# ❌ Bad: Mixed responsibilities
robot_stuff/        # Everything in one package
  ├── sensors
  ├── planning
  └── vision
```

### 3. Proper Dependency Management

```xml
<!-- ✅ Good: Explicit, minimal dependencies -->
<depend>rclpy</depend>
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>

<!-- ❌ Bad: Over-specifying or circular dependencies -->
<depend>entire_robot_stack</depend>  <!-- Too broad -->
```

### 4. Use Namespaces

```python
# ✅ Good: Topic in namespace
self.publisher = self.create_publisher(
    LaserScan,
    'scan',  # Will be /namespace/scan
    10
)

# ❌ Bad: Global topic names
self.publisher = self.create_publisher(
    LaserScan,
    '/scan',  # Forces global namespace
    10
)
```

### 5. Parameter-Driven Configuration

```python
# ✅ Good: Configurable via parameters
self.declare_parameter('update_rate', 10.0)
self.update_rate = self.get_parameter('update_rate').value

# ❌ Bad: Hard-coded values
self.update_rate = 10.0  # Requires code change to modify
```

### 6. Documentation

```python
# ✅ Good: Docstrings and comments
def process_data(self, data):
    """
    Process sensor data and extract features.

    Args:
        data: Raw sensor readings (list of floats)

    Returns:
        Processed feature vector (numpy array)
    """
    # Implementation
```

### 7. Error Handling

```python
# ✅ Good: Handle errors gracefully
try:
    result = self.risky_operation()
except Exception as e:
    self.get_logger().error(f'Operation failed: {e}')
    return None

# ❌ Bad: Let exceptions crash the node
result = self.risky_operation()  # May crash
```

### 8. Logging Best Practices

```python
# ✅ Good: Appropriate log levels
self.get_logger().debug('Detailed debug info')
self.get_logger().info('Normal operation message')
self.get_logger().warn('Something unusual happened')
self.get_logger().error('Recoverable error occurred')
self.get_logger().fatal('Unrecoverable error, shutting down')

# ❌ Bad: Everything is INFO
self.get_logger().info('Debug info')
self.get_logger().info('Error occurred')
```

### 9. Resource Cleanup

```python
# ✅ Good: Proper cleanup in destructor
def __del__(self):
    if hasattr(self, 'serial_port'):
        self.serial_port.close()

# Or use context managers
def main():
    rclpy.init()
    node = MyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### 10. Version Control

```bash
# ✅ Good: .gitignore for ROS 2 workspace
build/
install/
log/
*.pyc
__pycache__/
.vscode/
*.bag

# Include in version control
src/
README.md
```

---

## Workspace Organization

### Recommended Workspace Structure

```
ros2_ws/
├── src/
│   ├── robot_core/          # Core robot packages
│   │   ├── robot_bringup/   # Launch files, configs
│   │   ├── robot_description/  # URDF, meshes
│   │   └── robot_msgs/      # Custom message definitions
│   ├── robot_perception/    # Perception packages
│   │   ├── camera_driver/
│   │   ├── lidar_driver/
│   │   └── object_detection/
│   ├── robot_navigation/    # Navigation packages
│   │   ├── path_planning/
│   │   └── localization/
│   └── robot_manipulation/  # Manipulation packages
│       ├── arm_controller/
│       └── gripper_driver/
├── build/
├── install/
└── log/
```

---

## Common colcon Commands

```bash
# Build commands
colcon build                              # Build all packages
colcon build --symlink-install            # Symlink Python files (faster iteration)
colcon build --packages-select pkg1 pkg2  # Build specific packages
colcon build --packages-up-to pkg1        # Build pkg1 and its dependencies
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release  # Release build

# Test commands
colcon test                               # Run all tests
colcon test --packages-select pkg1        # Test specific package
colcon test-result --all                  # Show test results
colcon test-result --verbose              # Detailed test output

# Clean commands
colcon build --cmake-clean-cache          # Clean CMake cache
rm -rf build/ install/ log/               # Complete clean

# Info commands
colcon list                               # List all packages
colcon list --packages-up-to pkg1         # List pkg1 dependencies
colcon graph                              # Show dependency graph
```

---

## Troubleshooting

### Problem: Package not found after building

```bash
# Solution: Source the workspace
source ~/ros2_ws/install/setup.bash

# Verify package is visible
ros2 pkg list | grep my_package
```

### Problem: Changes not reflected after rebuild

```bash
# Solution: Clean build and rebuild
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Problem: Import errors in Python nodes

```bash
# Solution: Ensure package structure is correct
# Check that __init__.py exists
ls src/my_pkg/my_pkg/__init__.py

# Verify entry points in setup.py
cat src/my_pkg/setup.py | grep entry_points
```

### Problem: Dependencies not found

```bash
# Solution: Install missing dependencies
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## Key Takeaways

1. **Packages are the fundamental unit** of ROS 2 software organization
2. **package.xml defines metadata** and dependencies explicitly
3. **setup.py configures Python packages** and registers executables
4. **colcon is the build tool** - use `--symlink-install` for Python
5. **Source the workspace** after building to use packages
6. **Follow naming conventions** - lowercase with underscores
7. **One responsibility per package** - modularity enables reuse
8. **Test your code** - unit tests, integration tests, launch tests
9. **Document everything** - README, docstrings, comments
10. **Version control wisely** - ignore build artifacts, commit source

---

## Exercises

1. **Create a Complete Package**: Build a `robot_controller` package with:
   - A velocity controller node
   - Parameter configuration in YAML
   - Launch file to start the node
   - At least one unit test

2. **Multi-Package Workspace**: Create two packages:
   - `sensor_pkg`: Publishes simulated sensor data
   - `processor_pkg`: Subscribes to sensor data and processes it
   - Make `processor_pkg` depend on `sensor_pkg`
   - Build both and verify they communicate

3. **Dependency Management**:
   - Create a package that uses `numpy` and `scipy`
   - Add proper dependencies to `package.xml`
   - Verify it builds on a fresh system using `rosdep`

4. **Testing Practice**:
   - Write 3 unit tests for one of your nodes
   - Write 1 integration test that launches a node and checks topic output
   - Run tests with `colcon test` and verify they pass

5. **Best Practices Audit**:
   - Review one of your existing packages
   - Apply at least 5 best practices from this chapter
   - Document the improvements

---

## Next: Module 2 - Robot Simulation

Congratulations on completing Module 1! You now have a solid foundation in ROS 2:
- Week 3: Nodes, topics, services, actions
- Week 4: Launch files and parameters
- Week 5: Packages and best practices

In **Module 2**, we'll learn how to simulate robots safely before deploying to hardware:
- Gazebo for physics simulation
- URDF for robot description
- Unity for advanced visualization

Continue to [Module 2: Digital Twin (Gazebo & Unity)](/docs/module-2/intro)
