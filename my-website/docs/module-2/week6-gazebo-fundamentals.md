---
sidebar_position: 2
title: Week 6 - Gazebo Fundamentals
---

# Week 6: Gazebo Fundamentals - Robot Simulation

**Learning Objectives:**
- Understand why simulation is critical for robotics development
- Create robot models using URDF (Unified Robot Description Format)
- Build simulation worlds with SDF (Simulation Description Format)
- Integrate sensors (camera, LIDAR, IMU) in Gazebo
- Launch and control robots in Gazebo from ROS 2
- Visualize sensor data and debug simulations

---

## Why Simulate? The Case for Gazebo

Real robot development is **expensive, slow, and risky**:

**Real Robot Challenges:**
- 💰 **Cost**: Hardware ($10k-$1M+), maintenance, repairs
- ⏱️ **Time**: Setup takes hours, experiments are slow
- ⚠️ **Risk**: Crashes damage hardware, unsafe for testing edge cases
- 🔄 **Iteration**: Changing hardware design requires physical modifications
- 🌍 **Scale**: Can't test in diverse environments (Mars, underwater, hazardous zones)

**Simulation Advantages:**
- ✅ **Cost**: Free, infinite robots
- ✅ **Speed**: Parallel experiments, fast iteration
- ✅ **Safety**: Test crashes, failures, edge cases without risk
- ✅ **Reproducibility**: Exact same conditions every time
- ✅ **Accessibility**: Develop without physical hardware

### The Development Cycle

```
1. Design in CAD → 2. Simulate in Gazebo → 3. Test algorithms → 4. Deploy to real robot
         ↑                                                               ↓
         └────────────────── Refine based on real-world data ───────────┘
```

**Sim-to-Real Transfer**: Modern approaches use simulation for 90% of development, then fine-tune on real hardware.

---

## What is Gazebo?

**Gazebo** (formerly Gazebo Classic, now Gazebo Ignition) is an open-source 3D robotics simulator that provides:

1. **Physics Simulation**: Realistic dynamics (gravity, friction, inertia)
   - Engines: ODE (default), Bullet, Simbody, DART
2. **Sensor Simulation**: Camera, LIDAR, IMU, GPS, force/torque
3. **ROS 2 Integration**: Seamless communication via `gazebo_ros_pkgs`
4. **Visualization**: Real-time 3D rendering
5. **Plugins**: Extensible architecture for custom behaviors

**Gazebo vs Alternatives:**

| Feature | Gazebo | PyBullet | Isaac Sim (Week 8) |
|---------|--------|----------|-------------------|
| **Physics** | ODE/Bullet/Simbody | Bullet | PhysX |
| **Graphics** | Good | Minimal | Photorealistic |
| **ROS Integration** | Native | Manual | Native (ROS 2) |
| **Use Case** | General robotics | Research/ML | NVIDIA GPUs, AI |
| **Learning Curve** | Medium | Low | High |

**When to use Gazebo**: General-purpose robotics (mobile robots, manipulators, humanoids) with strong ROS 2 integration.

---

## URDF: Unified Robot Description Format

**URDF** is an XML format that describes robot **structure** (geometry, mass, joints).

### Core Concepts

A robot in URDF is a **tree of links connected by joints**:

```
Base Link (fixed to world)
    ↓ (joint: revolute)
Link 1
    ↓ (joint: revolute)
Link 2 (end effector)
```

**Key Elements:**
1. **Links**: Rigid bodies (visual, collision, inertial properties)
2. **Joints**: Connections between links (revolute, prismatic, fixed)
3. **Sensors**: Attached to links (camera, LIDAR)
4. **Materials**: Colors and textures

### Example: Simple 2-Link Robot Arm

```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <inertia ixx="0.00104" ixy="0" ixz="0"
               iyy="0.00104" iyz="0" izz="0.00125"/>
    </inertial>
  </link>

  <!-- Link 1 (First arm segment) -->
  <link name="link1">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <inertia ixx="0.00375" ixy="0" ixz="0"
               iyy="0.00375" iyz="0" izz="0.000104"/>
    </inertial>
  </link>

  <!-- Joint 1 (Base to Link1) -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>  <!-- Rotate around Z-axis -->
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

  <!-- Link 2 (Second arm segment) -->
  <link name="link2">
    <visual>
      <geometry>
        <box size="0.04 0.04 0.25"/>
      </geometry>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.04 0.04 0.25"/>
      </geometry>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <inertia ixx="0.00156" ixy="0" ixz="0"
               iyy="0.00156" iyz="0" izz="0.000040"/>
    </inertial>
  </link>

  <!-- Joint 2 (Link1 to Link2) -->
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>  <!-- Rotate around Y-axis -->
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>
</robot>
```

**Explanation:**
- **`<link>`**: Defines a rigid body with:
  - `<visual>`: Appearance (color, shape)
  - `<collision>`: Simplified shape for physics
  - `<inertial>`: Mass and inertia tensor (required for physics)
- **`<joint>`**: Connects two links
  - `type="revolute"`: Rotating joint (like an elbow)
  - `<limit>`: Joint range (radians), max effort (N·m), max velocity (rad/s)
  - `<axis>`: Rotation axis (0 0 1 = Z-axis)

### Computing Inertia Tensors

Inertia tensors are required for realistic physics. For simple shapes:

```python
import numpy as np

def box_inertia(mass, width, depth, height):
    """Inertia tensor for a box (centered at origin)"""
    ixx = (1/12) * mass * (depth**2 + height**2)
    iyy = (1/12) * mass * (width**2 + height**2)
    izz = (1/12) * mass * (width**2 + depth**2)
    return {'ixx': ixx, 'iyy': iyy, 'izz': izz, 'ixy': 0, 'ixz': 0, 'iyz': 0}

def cylinder_inertia(mass, radius, length):
    """Inertia tensor for a cylinder (z-axis aligned)"""
    ixx = (1/12) * mass * (3*radius**2 + length**2)
    iyy = ixx
    izz = (1/2) * mass * radius**2
    return {'ixx': ixx, 'iyy': iyy, 'izz': izz, 'ixy': 0, 'ixz': 0, 'iyz': 0}

def sphere_inertia(mass, radius):
    """Inertia tensor for a sphere"""
    i = (2/5) * mass * radius**2
    return {'ixx': i, 'iyy': i, 'izz': i, 'ixy': 0, 'ixz': 0, 'iyz': 0}

# Example usage
link1_mass = 0.5
link1_dims = (0.05, 0.05, 0.3)  # width, depth, height
inertia = box_inertia(link1_mass, *link1_dims)
print(f"Link1 inertia: ixx={inertia['ixx']:.6f}, iyy={inertia['iyy']:.6f}, izz={inertia['izz']:.6f}")
# Output: Link1 inertia: ixx=0.003750, iyy=0.003750, izz=0.000104
```

**Pro Tip**: CAD software (SolidWorks, Fusion 360) can export URDF with accurate inertia. For hand-written URDF, use the formulas above.

---

## SDF: Simulation Description Format

**SDF** (Simulation Description Format) describes **worlds** (environments, lighting, physics parameters). While URDF describes a single robot, SDF describes entire scenes.

### SDF vs URDF

| Feature | URDF | SDF |
|---------|------|-----|
| **Purpose** | Robot structure | World/scene description |
| **Models** | Single robot | Multiple robots, objects |
| **Physics** | No (assumes defaults) | Yes (gravity, solver params) |
| **ROS Integration** | Native | Via `robot_state_publisher` |
| **Complexity** | Simpler | More features |

### Example: Gazebo World with Robot

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="simple_world">
    <!-- Physics Settings -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.5 -1</direction>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Obstacle (Box) -->
    <model name="box_obstacle">
      <pose>2 0 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Include Robot (from URDF) -->
    <include>
      <uri>model://simple_arm</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>

  </world>
</sdf>
```

**Key Elements:**
- **`<physics>`**: Solver settings (time step, real-time factor)
- **`<light>`**: Directional/point/spot lights
- **`<model>`**: Static (obstacles) or dynamic (robots)
- **`<include>`**: Load external models (from Gazebo model database or custom)

---

## Adding Sensors to URDF

Sensors are added as **Gazebo plugins** attached to links.

### Camera Sensor

```xml
<!-- Add to robot URDF -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.0001" ixy="0" ixz="0"
             iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0 0.15" rpy="0 0 0"/>
</joint>

<!-- Gazebo Plugin for Camera -->
<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>0.0</updateRate>
      <cameraName>robot/camera</cameraName>
      <imageTopicName>image_raw</imageTopicName>
      <cameraInfoTopicName>camera_info</cameraInfoTopicName>
      <frameName>camera_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```

**Explanation:**
- **`<sensor type="camera">`**: Defines camera parameters (FOV, resolution)
- **`<plugin>`**: `libgazebo_ros_camera.so` publishes images to ROS 2 topic
- **Topics published**:
  - `/robot/camera/image_raw` (sensor_msgs/Image)
  - `/robot/camera/camera_info` (sensor_msgs/CameraInfo)

### LIDAR Sensor

```xml
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder length="0.05" radius="0.05"/>
    </geometry>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1"/>
    </material>
  </visual>
  <inertial>
    <mass value="0.2"/>
    <inertia ixx="0.0001" ixy="0" ixz="0"
             iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0 0 0.2" rpy="0 0 0"/>
</joint>

<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="gazebo_ros_laser" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Output**: Publishes to `/robot/scan` (sensor_msgs/LaserScan) with 360 range measurements.

### IMU Sensor

```xml
<gazebo reference="base_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=imu</remapping>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</gazebo>
```

**Output**: Publishes to `/robot/imu` (sensor_msgs/Imu) with orientation, angular velocity, linear acceleration.

---

## Launching Gazebo from ROS 2

### Basic Launch File

```python
# robot_sim.launch.py
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot_description = get_package_share_directory('my_robot_description')

    urdf_file = os.path.join(pkg_robot_description, 'urdf', 'simple_arm.urdf')
    world_file = os.path.join(pkg_robot_description, 'worlds', 'simple_world.world')

    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        # Launch Gazebo with world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ]),
            launch_arguments={'world': world_file}.items()
        ),

        # Robot State Publisher (publishes robot TF tree)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # Spawn Robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'simple_arm',
                '-topic', 'robot_description',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1'
            ],
            output='screen'
        ),

        # Joint State Publisher (for manual control in RViz)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
    ])
```

**Run it:**
```bash
ros2 launch my_robot_description robot_sim.launch.py
```

**What happens:**
1. Gazebo starts with the specified world
2. `robot_state_publisher` publishes TF transforms from URDF
3. `spawn_entity.py` inserts the robot into Gazebo at (x=0, y=0, z=0.1)
4. Joint State Publisher GUI opens for manual joint control

---

## Controlling the Robot

### Publishing Joint Commands

```python
# joint_commander.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class JointCommander(Node):
    def __init__(self):
        super().__init__('joint_commander')

        # Publisher for joint position commands
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10
        )

        # Timer for periodic commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.time = 0.0

    def timer_callback(self):
        """Send sinusoidal joint commands"""
        msg = Float64MultiArray()

        # Joint 1: Oscillate around Z-axis
        joint1_pos = 0.5 * math.sin(self.time)

        # Joint 2: Oscillate around Y-axis (phase shifted)
        joint2_pos = 0.3 * math.cos(self.time + 1.0)

        msg.data = [joint1_pos, joint2_pos]
        self.pub.publish(msg)

        self.get_logger().info(f'Commands: J1={joint1_pos:.2f}, J2={joint2_pos:.2f}')
        self.time += 0.1

def main(args=None):
    rclpy.init(args=args)
    node = JointCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run it:**
```bash
ros2 run my_robot_control joint_commander
```

**Prerequisites**: You need a controller plugin in your URDF:

```xml
<gazebo>
  <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    <robot_param>robot_description</robot_param>
    <robot_param_node>robot_state_publisher</robot_param_node>
    <parameters>$(find my_robot_description)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>
```

---

## Subscribing to Sensor Data

### Camera Subscriber

```python
# camera_viewer.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/robot/camera/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info('Camera viewer started')

    def image_callback(self, msg):
        """Display camera image"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Display
            cv2.imshow("Robot Camera", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Dependencies**: `sudo apt install ros-humble-cv-bridge python3-opencv`

### LIDAR Subscriber

```python
# lidar_processor.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')

        self.subscription = self.create_subscription(
            LaserScan,
            '/robot/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        """Process LIDAR data"""
        ranges = np.array(msg.ranges)

        # Replace inf with max range
        ranges[np.isinf(ranges)] = msg.range_max

        # Find closest obstacle
        min_distance = np.min(ranges)
        min_angle_idx = np.argmin(ranges)
        min_angle = msg.angle_min + min_angle_idx * msg.angle_increment

        self.get_logger().info(
            f'Closest obstacle: {min_distance:.2f}m at {np.degrees(min_angle):.1f}°'
        )

        # Check for obstacles in front (±15 degrees)
        front_indices = np.where(np.abs(np.linspace(msg.angle_min, msg.angle_max, len(ranges))) < 0.26)[0]
        front_ranges = ranges[front_indices]

        if np.min(front_ranges) < 1.0:
            self.get_logger().warn('OBSTACLE AHEAD!')

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Example: Mobile Robot with Differential Drive

Let's create a complete mobile robot with differential drive (like a vacuum cleaner).

### URDF: Mobile Robot

```xml
<?xml version="1.0"?>
<robot name="mobile_robot">
  <!-- Base Link (chassis) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.0417" ixy="0" ixz="0"
               iyy="0.0729" iyz="0" izz="0.1042"/>
    </inertial>
  </link>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.00260" ixy="0" ixz="0"
               iyy="0.00260" iyz="0" izz="0.00500"/>
    </inertial>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.175 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Right Wheel (symmetric) -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.00260" ixy="0" ixz="0"
               iyy="0.00260" iyz="0" izz="0.00500"/>
    </inertial>
  </link>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.175 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Caster Wheel (front support) -->
  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.0005" ixy="0" ixz="0"
               iyy="0.0005" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <joint name="caster_wheel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="0.15 0 -0.1" rpy="0 0 0"/>
  </joint>

  <!-- Differential Drive Plugin -->
  <gazebo>
    <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <update_rate>50</update_rate>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.35</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <command_topic>cmd_vel</command_topic>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>

</robot>
```

### Controlling the Mobile Robot

```python
# mobile_robot_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class MobileRobotController(Node):
    def __init__(self):
        super().__init__('mobile_robot_controller')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.time = 0.0

    def timer_callback(self):
        """Drive in a circle"""
        msg = Twist()

        # Linear velocity (forward)
        msg.linear.x = 0.5  # m/s

        # Angular velocity (turning)
        msg.angular.z = 0.5  # rad/s

        self.pub.publish(msg)
        self.time += 0.1

        # Stop after 10 seconds
        if self.time > 10.0:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.pub.publish(msg)
            self.get_logger().info('Stopped')

def main(args=None):
    rclpy.init(args=args)
    node = MobileRobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run it:**
```bash
# Terminal 1: Launch simulation
ros2 launch my_robot_description mobile_robot_sim.launch.py

# Terminal 2: Control the robot
ros2 run my_robot_control mobile_robot_controller

# Terminal 3: Visualize in RViz
ros2 run rviz2 rviz2
```

---

## Debugging and Visualization

### Viewing TF Tree

```bash
# Install if needed
sudo apt install ros-humble-tf2-tools

# View TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo base_link camera_link
```

### Listing Topics

```bash
# See all topics
ros2 topic list

# Inspect topic data
ros2 topic echo /robot/camera/image_raw

# Check message frequency
ros2 topic hz /robot/scan
```

### RViz Configuration

1. Open RViz: `ros2 run rviz2 rviz2`
2. Add displays:
   - **RobotModel**: Shows URDF
   - **Camera**: Subscribe to `/robot/camera/image_raw`
   - **LaserScan**: Subscribe to `/robot/scan`
   - **TF**: Shows coordinate frames
3. Set **Fixed Frame** to `base_link` or `odom`

---

## Key Takeaways

1. **Simulation accelerates development** - Test safely, iterate quickly, no hardware needed
2. **URDF describes robot structure** - Links (rigid bodies) + joints (connections)
3. **Inertia tensors are critical** - Required for realistic physics (use formulas or CAD export)
4. **SDF describes worlds** - Physics settings, lighting, multiple models
5. **Gazebo plugins enable ROS 2 integration** - Sensors publish to topics, controllers subscribe
6. **Sensors are simulated realistically** - Camera, LIDAR, IMU with noise and limitations
7. **Differential drive is common for mobile robots** - `cmd_vel` topic controls motion
8. **Debugging tools are essential** - TF tree, topic inspection, RViz visualization

---

## Exercises

### Conceptual

1. **Why simulate?** List 3 scenarios where simulation is preferable to real hardware testing, and 1 scenario where it's not.

2. **URDF vs SDF**: When would you use SDF instead of URDF? Give a concrete example.

### Coding

3. **Add a Depth Camera**: Modify the `simple_arm` URDF to add an Intel RealSense-like depth camera (RGB-D). Publish both color and depth images.

4. **Obstacle Avoidance**: Write a ROS 2 node that reads LIDAR data and publishes `cmd_vel` to avoid obstacles (stop if obstacle within 1m ahead).

5. **Humanoid Torso**: Create a simple humanoid torso URDF with:
   - Base (pelvis)
   - Spine joint (revolute, ±30°)
   - Two shoulders (revolute, ±90°)
   - Compute realistic inertia tensors

---

## Next: Week 7 - Unity Visualization

Gazebo is great for physics, but for **high-quality visualization, UI/UX, and human interaction**, we'll use **Unity** (Week 7). Unity provides:
- Photorealistic rendering
- Advanced animation (humanoid characters)
- GUI elements (buttons, sliders)
- VR/AR support

See you in [Week 7](./week7-unity-visualization)!

---

**Further Resources:**
- [Gazebo Tutorials](https://gazebosim.org/docs)
- [URDF Specification](http://wiki.ros.org/urdf/XML)
- [gazebo_ros_pkgs Documentation](https://github.com/ros-simulation/gazebo_ros_pkgs)
- [Inertia Calculator](https://amesweb.info/inertia/mass-moment-of-inertia-calculator.aspx)
