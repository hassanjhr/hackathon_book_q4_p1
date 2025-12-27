# Phase 2 Specification: Robot Simulation & Conversational AI

**Project:** Physical AI & Humanoid Robotics Textbook
**Specification Created:** 2025-12-16
**Phase:** 2 of 4
**Status:** Ready for Implementation

---

## Executive Summary

### Scope
Phase 2 expands the textbook with practical simulation tools and conversational AI integration:
- **Module 2 (Robot Simulation)**: Weeks 6-7
- **Module 4 (Conversational Robotics)**: Week 13

### Objectives
1. Enable students to simulate robots in Gazebo and Unity
2. Teach URDF/SDF for robot modeling
3. Integrate conversational AI (LLM + speech) with robotics
4. Provide hands-on examples with ROS 2 integration

### Constraints
- **Content-only changes** - No deployment, infrastructure, or configuration changes
- **Follow existing style** - Match Module 1 and Module 5 patterns
- **Beginner-friendly** - Intuition before complexity
- **Code-centric** - Runnable Python examples with comments
- **Self-contained** - Each week stands alone

### Success Criteria
- ✅ Build passes (`npm run build`)
- ✅ No broken links
- ✅ 5 files created/updated
- ✅ Consistent style with Phase 1
- ✅ All code examples runnable

---

## Module 2: Robot Simulation (Weeks 6-7)

### Overview
**Duration:** Weeks 6-7
**Goal:** Teach robot simulation for testing before real hardware deployment
**Prerequisites:** Module 1 (ROS 2 Fundamentals)

### Learning Path
```
Week 6: Gazebo Fundamentals
  ↓
Week 7: Unity Visualization
  ↓
Module 3: NVIDIA Isaac Platform
```

---

## Task 2.1: Week 6 - Gazebo Fundamentals

### File Location
`/my-website/docs/module-2/week6-gazebo-fundamentals.md`

### Frontmatter
```yaml
---
sidebar_position: 2
title: Week 6 - Gazebo Fundamentals
---
```

### Learning Objectives (6 items)
1. Install and configure Gazebo simulator with ROS 2
2. Write URDF files for robot description
3. Understand SDF for world and environment modeling
4. Configure physics engines (ODE, Bullet, Simbody)
5. Simulate sensors (camera, LIDAR, IMU)
6. Launch robots in Gazebo using ROS 2 launch files

### Content Structure

#### 1. What is Gazebo?
**Intuition:**
- Robot simulation before hardware
- Test algorithms safely and quickly
- Physics-based simulation (gravity, collisions, friction)
- Multi-robot scenarios

**Architecture:**
```
Gazebo Server (Physics)
    ↓
Gazebo Client (Visualization)
    ↓
ROS 2 Bridge (Topics, Services, Actions)
    ↓
Your Robot Code
```

**Use Cases:**
- Algorithm development (navigation, manipulation)
- Sensor testing (LIDAR, camera, depth)
- Multi-robot coordination
- Edge case testing (collisions, failures)

#### 2. Installing Gazebo with ROS 2
**Code Example 1: Installation**
```bash
# Install Gazebo Classic (ROS 2 Humble)
sudo apt install ros-humble-gazebo-ros-pkgs

# Or install new Gazebo (Ignition)
sudo apt install ros-humble-ros-gz

# Test installation
gazebo --version
gz sim --version
```

**Code Example 2: Launch Empty World**
```bash
ros2 launch gazebo_ros gazebo.launch.py
```

#### 3. URDF (Unified Robot Description Format)
**Intuition:**
- XML file describing robot structure
- Links (rigid bodies) + Joints (connections)
- Visual (what you see) + Collision (what hits) + Inertial (mass)

**URDF Structure:**
```xml
<robot name="my_robot">
  <link name="base_link">
    <visual>...</visual>
    <collision>...</collision>
    <inertial>...</inertial>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>

  <link name="link1">
    ...
  </link>
</robot>
```

**Code Example 3: Simple 2-Link Robot Arm URDF**
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
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joint 1 (Base to Link 1) -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10.0" velocity="1.0"/>
  </joint>

  <!-- Link 1 -->
  <link name="link1">
    <visual>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.03"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint 2 (Link 1 to Link 2) -->
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10.0" velocity="1.0"/>
  </joint>

  <!-- Link 2 -->
  <link name="link2">
    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.02"/>
      </geometry>
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

</robot>
```

**Save as:** `simple_arm.urdf`

**Code Example 4: View URDF in RViz**
```bash
# Install joint_state_publisher_gui
sudo apt install ros-humble-joint-state-publisher-gui

# View URDF
ros2 launch urdf_tutorial display.launch.py model:=simple_arm.urdf
```

#### 4. Joint Types and Properties
**Joint Types:**
- **revolute**: Rotation with limits (e.g., elbow)
- **continuous**: Rotation without limits (e.g., wheel)
- **prismatic**: Linear motion (e.g., elevator)
- **fixed**: No motion (e.g., camera mount)
- **planar**: 2D motion
- **floating**: 6DOF (rarely used)

**Code Example 5: Different Joint Types**
```xml
<!-- Revolute: Elbow joint -->
<joint name="elbow" type="revolute">
  <axis xyz="0 1 0"/>
  <limit lower="-2.0" upper="2.0" effort="50" velocity="2"/>
</joint>

<!-- Continuous: Wheel -->
<joint name="wheel_joint" type="continuous">
  <axis xyz="0 0 1"/>
</joint>

<!-- Prismatic: Linear actuator -->
<joint name="slider" type="prismatic">
  <axis xyz="1 0 0"/>
  <limit lower="0" upper="0.5" effort="100" velocity="0.5"/>
</joint>
```

#### 5. Inertial Properties
**Why Inertia Matters:**
- Physics simulation requires mass and inertia
- Affects dynamics (how robot moves under forces)
- Poor inertia = unstable simulation

**Code Example 6: Calculate Inertia for Simple Shapes**
```python
import numpy as np

def cylinder_inertia(mass, radius, length):
    """Inertia tensor for a cylinder (z-axis aligned)"""
    ixx = (1/12) * mass * (3*radius**2 + length**2)
    iyy = ixx
    izz = (1/2) * mass * radius**2
    return {'ixx': ixx, 'iyy': iyy, 'izz': izz, 'ixy': 0, 'ixz': 0, 'iyz': 0}

def box_inertia(mass, width, depth, height):
    """Inertia tensor for a box"""
    ixx = (1/12) * mass * (depth**2 + height**2)
    iyy = (1/12) * mass * (width**2 + height**2)
    izz = (1/12) * mass * (width**2 + depth**2)
    return {'ixx': ixx, 'iyy': iyy, 'izz': izz, 'ixy': 0, 'ixz': 0, 'iyz': 0}

def sphere_inertia(mass, radius):
    """Inertia tensor for a sphere"""
    i = (2/5) * mass * radius**2
    return {'ixx': i, 'iyy': i, 'izz': i, 'ixy': 0, 'ixz': 0, 'iyz': 0}

# Example: Cylinder link
mass = 0.5  # kg
radius = 0.03  # m
length = 0.5  # m

inertia = cylinder_inertia(mass, radius, length)
print(f"Cylinder inertia: {inertia}")
# Output: {'ixx': 0.0104, 'iyy': 0.0104, 'izz': 0.000225, ...}
```

#### 6. SDF (Simulation Description Format)
**URDF vs SDF:**
| Feature | URDF | SDF |
|---------|------|-----|
| Format | XML | XML |
| Purpose | Robot description | World + robot |
| Physics | Limited | Full control |
| Sensors | Via plugins | Native support |
| Use Case | ROS integration | Gazebo-native |

**Code Example 7: Simple SDF World**
```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="simple_world">

    <!-- Physics -->
    <physics type="ode">
      <gravity>0 0 -9.81</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Sun light -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Simple box obstacle -->
    <model name="box">
      <pose>2 0 0.5 0 0 0</pose>
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

  </world>
</sdf>
```

**Save as:** `simple_world.sdf`

#### 7. Sensor Plugins
**Common Sensors:**
- Camera (RGB, Depth)
- LIDAR (2D/3D)
- IMU (Inertial Measurement Unit)
- GPS
- Contact sensors

**Code Example 8: Camera Sensor in URDF**
```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/camera</namespace>
        <remapping>~/image_raw:=image_raw</remapping>
        <remapping>~/camera_info:=camera_info</remapping>
      </ros>
      <camera_name>front_camera</camera_name>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Code Example 9: LIDAR Sensor in URDF**
```xml
<gazebo reference="lidar_link">
  <sensor name="lidar" type="ray">
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
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/lidar</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

#### 8. Launch Robot in Gazebo
**Code Example 10: Complete Launch File**
```python
# spawn_robot.launch.py
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to URDF
    urdf_file = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        'simple_arm.urdf'
    )

    # Read URDF content
    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    return LaunchDescription([
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'),
                            'launch', 'gazebo.launch.py')
            ]),
            launch_arguments={'world': 'simple_world.sdf'}.items()
        ),

        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'simple_arm',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.5'
            ],
            output='screen'
        ),

        # Publish robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
    ])
```

**Launch:**
```bash
ros2 launch my_robot_description spawn_robot.launch.py
```

#### 9. Reading Sensor Data
**Code Example 11: Subscribe to Camera Topic**
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Display image
        cv2.imshow('Gazebo Camera', cv_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = CameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 10. Complete Example: Mobile Robot
**Code Example 12: Differential Drive Robot URDF** (Simplified)
```xml
<?xml version="1.0"?>
<robot name="mobile_robot">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.175 0" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Right Wheel (symmetric) -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.175 0" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Differential Drive Plugin -->
  <gazebo>
    <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace>/</namespace>
      </ros>
      <update_rate>50</update_rate>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.35</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
    </plugin>
  </gazebo>

</robot>
```

**Control the robot:**
```bash
# Spawn robot in Gazebo
ros2 launch my_robot_description spawn_mobile_robot.launch.py

# Drive robot forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" --once
```

### Key Takeaways (8 items)
1. **Gazebo** simulates physics-based robot behavior before hardware deployment
2. **URDF** describes robot structure: links (bodies) + joints (connections)
3. **Links** have visual, collision, and inertial properties - all three are required for simulation
4. **Joints** connect links and define motion constraints (revolute, prismatic, continuous)
5. **Inertial properties** (mass, inertia tensor) affect dynamics - calculate correctly for realistic simulation
6. **SDF** extends URDF with world/environment modeling and advanced physics configuration
7. **Sensor plugins** (camera, LIDAR, IMU) publish to ROS 2 topics - same interface as real sensors
8. **Differential drive plugin** enables robot mobility without writing low-level controllers

### Exercises (5 questions)

#### Conceptual (2)
1. **URDF Structure**: Draw the link-joint tree for a humanoid arm (shoulder, elbow, wrist, hand). Label each joint type and DOF.

2. **Sensor Comparison**: Compare camera vs LIDAR sensors in Gazebo:
   - Data type and format
   - Computational cost
   - Use cases (when to use each)
   - Limitations in simulation

#### Coding (3)
3. **3-Link Planar Arm**: Create a URDF for a 3-link planar robot arm:
   - Base link (fixed)
   - 3 revolute joints (all rotating around same axis)
   - Link lengths: 1.0m, 0.8m, 0.5m
   - Add visual, collision, and inertial properties
   - Spawn in Gazebo and control with `joint_state_publisher_gui`

4. **Mobile Robot with Camera**: Extend the mobile robot URDF:
   - Add a camera sensor mounted on top (0.2m above base)
   - Configure camera: 640x480 resolution, 60° FOV
   - Write a ROS 2 node to subscribe to camera images and save them as JPEGs

5. **Obstacle Course World**: Create an SDF world file with:
   - Flat ground plane
   - 5 randomly placed box obstacles (varying sizes)
   - A goal marker (green sphere)
   - Spawn a mobile robot and navigate around obstacles using teleop

### Next Steps
- **Week 7**: Learn Unity visualization for high-quality graphics and VR/AR
- **Resources**:
  - [Gazebo Tutorials](http://gazebosim.org/tutorials)
  - [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
  - [gazebo_ros_pkgs Documentation](http://gazebosim.org/tutorials?tut=ros2_overview)

---

## Task 2.2: Week 7 - Unity Visualization

### File Location
`/my-website/docs/module-2/week7-unity-visualization.md`

### Frontmatter
```yaml
---
sidebar_position: 3
title: Week 7 - Unity Visualization
---
```

### Learning Objectives (5 items)
1. Install Unity for robotics development
2. Set up ROS-TCP-Connector for Unity ↔ ROS 2 communication
3. Visualize robot motion in real-time from ROS 2 topics
4. Create humanoid animations with Unity's animation system
5. Compare Gazebo vs Unity workflows and choose appropriate tools

### Content Structure

#### 1. Why Unity for Robotics?
**Intuition:**
- Gazebo = Physics + Simulation
- Unity = Visualization + User Experience
- Use together: Gazebo for testing, Unity for demos/UI

**Unity Advantages:**
- **High-quality graphics**: Realistic rendering, shadows, lighting
- **AR/VR support**: Immersive robot control interfaces
- **UI/UX tools**: Buttons, sliders, dashboards
- **Cross-platform**: Desktop, mobile, web
- **Animation system**: Smooth humanoid motion

**Use Cases:**
- Digital twins (mirror real robot state)
- Virtual reality teleoperation
- Public demonstrations and marketing
- User interface development
- Training data generation (synthetic images)

#### 2. Gazebo vs Unity Comparison
| Feature | Gazebo | Unity |
|---------|--------|-------|
| **Primary Purpose** | Physics simulation | Visualization & UX |
| **Physics** | ODE, Bullet, Simbody | PhysX (basic) |
| **Graphics** | OGRE (basic) | High-quality rendering |
| **ROS Integration** | Native | Via ROS-TCP-Connector |
| **Sensors** | Accurate simulation | Visual/synthetic data |
| **Performance** | CPU-heavy | GPU-accelerated |
| **AR/VR** | No | Excellent |
| **UI Development** | Limited | Rich UI toolkit |
| **Learning Curve** | Moderate | Steep |
| **Best For** | Algorithm testing | Demos, HRI, visualization |

**Decision Guide:**
- **Use Gazebo** for: Navigation, planning, multi-robot sim, sensor testing
- **Use Unity** for: HRI, VR teleoperation, digital twins, public demos
- **Use Both** for: Gazebo simulation + Unity visualization (separate processes)

#### 3. Installing Unity
**Code Example 1: Unity Installation (Ubuntu)**
```bash
# Download Unity Hub (official way to install Unity)
wget https://public-cdn.cloud.unity3d.com/hub/prod/UnityHub.AppImage
chmod +x UnityHub.AppImage
./UnityHub.AppImage

# Or via snap
sudo snap install unity-hub --classic

# Install Unity Editor (version 2021.3 LTS recommended for robotics)
# Use Unity Hub to install:
# - Unity Editor 2021.3 LTS
# - Linux Build Support
# - Visual Studio Code Editor (optional)
```

**System Requirements:**
- Ubuntu 20.04+ or Windows 10+
- GPU with OpenGL 4.5+ or DirectX 11+
- 8GB RAM minimum (16GB recommended)
- 20GB disk space for Unity + projects

#### 4. ROS-TCP-Connector Setup
**Architecture:**
```
ROS 2 Node (Python/C++)
    ↓ (TCP/IP)
ROS-TCP-Endpoint (ROS 2 package)
    ↓ (Network socket)
Unity ROS-TCP-Connector (C# package)
    ↓
Unity Scene (GameObjects)
```

**Code Example 2: Install ROS-TCP-Endpoint (ROS 2 side)**
```bash
# Clone ROS-TCP-Endpoint
cd ~/ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git

# Build
cd ~/ros2_ws
colcon build --packages-select ros_tcp_endpoint

# Source
source install/setup.bash

# Launch endpoint (default port 10000)
ros2 run ros_tcp_endpoint default_server_endpoint
```

**Code Example 3: Install ROS-TCP-Connector (Unity side)**
```
1. Open Unity Editor
2. Window → Package Manager
3. Click "+" → "Add package from git URL"
4. Enter: https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
5. Click "Add"
6. Robotics → ROS Settings
   - ROS IP Address: 127.0.0.1 (or your ROS machine IP)
   - ROS Port: 10000
   - Protocol: ROS 2
```

#### 5. First Unity Project: Visualize Robot Pose
**Code Example 4: ROS 2 Publisher (Python)**
```python
# publish_pose.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import math

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher = self.create_publisher(Pose, '/robot_pose', 10)
        self.timer = self.create_timer(0.1, self.publish_pose)
        self.angle = 0.0

    def publish_pose(self):
        msg = Pose()
        # Circular motion
        radius = 2.0
        msg.position.x = radius * math.cos(self.angle)
        msg.position.y = radius * math.sin(self.angle)
        msg.position.z = 0.5
        msg.orientation.w = 1.0  # No rotation

        self.publisher.publish(msg)
        self.angle += 0.05

        if self.angle > 2 * math.pi:
            self.angle = 0.0

def main():
    rclpy.init()
    node = PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run:**
```bash
# Terminal 1: ROS-TCP-Endpoint
ros2 run ros_tcp_endpoint default_server_endpoint

# Terminal 2: Pose publisher
python3 publish_pose.py
```

**Code Example 5: Unity C# Subscriber**
```csharp
// PoseSubscriber.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class PoseSubscriber : MonoBehaviour
{
    public GameObject robotObject;
    private ROSConnection ros;

    void Start()
    {
        // Get ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<PoseMsg>("/robot_pose");

        // Subscribe to /robot_pose topic
        ros.Subscribe<PoseMsg>("/robot_pose", PoseCallback);
    }

    void PoseCallback(PoseMsg poseMsg)
    {
        // Update robot position in Unity
        Vector3 position = new Vector3(
            (float)poseMsg.position.x,
            (float)poseMsg.position.z,  // ROS Z → Unity Y (up)
            (float)poseMsg.position.y   // ROS Y → Unity Z (forward)
        );
        robotObject.transform.position = position;

        // Update rotation (quaternion)
        Quaternion rotation = new Quaternion(
            (float)poseMsg.orientation.x,
            (float)poseMsg.orientation.z,
            (float)poseMsg.orientation.y,
            (float)poseMsg.orientation.w
        );
        robotObject.transform.rotation = rotation;
    }
}
```

**Unity Setup:**
1. Create a new Unity project
2. Create a cube (GameObject → 3D Object → Cube) to represent robot
3. Attach `PoseSubscriber.cs` script to cube
4. Assign cube to `robotObject` field in Inspector
5. Press Play - cube should move in a circle!

#### 6. Importing Robot Models
**URDF to Unity Workflow:**
1. **Option A: URDF Importer (Unity Package)**
```
Unity → Package Manager → Add from git URL
https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer

Assets → Import Robot from URDF
Select .urdf file → Import
```

2. **Option B: Export from Blender/CAD**
- Export robot mesh as FBX or OBJ
- Import into Unity (drag into Assets folder)
- Manually configure articulations/joints

**Code Example 6: Articulation Body (Unity's Joint System)**
```csharp
// ArticulationController.cs
using UnityEngine;

public class ArticulationController : MonoBehaviour
{
    private ArticulationBody[] articulationBodies;

    void Start()
    {
        // Get all articulation bodies in robot
        articulationBodies = GetComponentsInChildren<ArticulationBody>();
    }

    public void SetJointPosition(int jointIndex, float targetPosition)
    {
        if (jointIndex < articulationBodies.Length)
        {
            ArticulationBody joint = articulationBodies[jointIndex];
            ArticulationDrive drive = joint.xDrive;
            drive.target = targetPosition * Mathf.Rad2Deg;  // Radians to degrees
            joint.xDrive = drive;
        }
    }

    public void SetJointVelocity(int jointIndex, float targetVelocity)
    {
        if (jointIndex < articulationBodies.Length)
        {
            ArticulationBody joint = articulationBodies[jointIndex];
            ArticulationDrive drive = joint.xDrive;
            drive.targetVelocity = targetVelocity * Mathf.Rad2Deg;
            joint.xDrive = drive;
        }
    }
}
```

#### 7. Joint State Visualization
**Code Example 7: Subscribe to JointState and Animate Robot**
```csharp
// JointStateSubscriber.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointStateSubscriber : MonoBehaviour
{
    private ROSConnection ros;
    private ArticulationBody[] joints;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>("/joint_states", JointStateCallback);

        // Get robot joints
        joints = GetComponentsInChildren<ArticulationBody>();
        Debug.Log($"Found {joints.Length} articulation bodies");
    }

    void JointStateCallback(JointStateMsg msg)
    {
        // Update each joint position
        for (int i = 0; i < msg.position.Length && i < joints.Length; i++)
        {
            ArticulationBody joint = joints[i];
            ArticulationDrive drive = joint.xDrive;

            // Convert radians to degrees (Unity uses degrees)
            drive.target = (float)msg.position[i] * Mathf.Rad2Deg;
            joint.xDrive = drive;
        }
    }
}
```

**ROS 2 Publisher (Python) - Publish joint states:**
```python
# publish_joint_states.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.05, self.publish_states)
        self.angle = 0.0

    def publish_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3']

        # Simple sinusoidal motion
        msg.position = [
            math.sin(self.angle),
            math.cos(self.angle),
            math.sin(self.angle * 2)
        ]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0]

        self.publisher.publish(msg)
        self.angle += 0.1

def main():
    rclpy.init()
    node = JointStatePublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

#### 8. Publishing from Unity to ROS 2
**Code Example 8: Unity Button Controls Robot**
```csharp
// UnityToROS.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class UnityToROS : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>("/cmd_vel");
    }

    public void MoveForward()
    {
        TwistMsg twist = new TwistMsg();
        twist.linear.x = 0.5;  // 0.5 m/s forward
        twist.angular.z = 0.0;
        ros.Publish("/cmd_vel", twist);
        Debug.Log("Sent: Move Forward");
    }

    public void TurnLeft()
    {
        TwistMsg twist = new TwistMsg();
        twist.linear.x = 0.0;
        twist.angular.z = 0.5;  // 0.5 rad/s counter-clockwise
        ros.Publish("/cmd_vel", twist);
        Debug.Log("Sent: Turn Left");
    }

    public void Stop()
    {
        TwistMsg twist = new TwistMsg();
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        ros.Publish("/cmd_vel", twist);
        Debug.Log("Sent: Stop");
    }
}
```

**Unity UI Setup:**
1. Create UI Canvas: GameObject → UI → Canvas
2. Add 3 buttons: "Forward", "Left", "Stop"
3. Attach `UnityToROS.cs` to an empty GameObject
4. Link button OnClick() events to respective functions

#### 9. Camera Feed: ROS 2 to Unity
**Code Example 9: Display ROS 2 Camera in Unity**
```csharp
// CameraSubscriber.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using UnityEngine.UI;

public class CameraSubscriber : MonoBehaviour
{
    public RawImage displayImage;  // Unity UI RawImage component
    private ROSConnection ros;
    private Texture2D texture;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<ImageMsg>("/camera/image_raw", ImageCallback);

        // Create texture (640x480 default)
        texture = new Texture2D(640, 480, TextureFormat.RGB24, false);
        displayImage.texture = texture;
    }

    void ImageCallback(ImageMsg msg)
    {
        // Resize texture if needed
        if (texture.width != (int)msg.width || texture.height != (int)msg.height)
        {
            texture.Resize((int)msg.width, (int)msg.height);
        }

        // Load image data (assume RGB8 encoding)
        texture.LoadRawTextureData(msg.data);
        texture.Apply();
    }
}
```

**Note:** Requires RawImage UI element and proper encoding handling (RGB8, RGBA8, etc.)

#### 10. Humanoid Animation Example
**Code Example 10: Animate Humanoid with Mecanim**
```csharp
// HumanoidAnimator.cs
using UnityEngine;

public class HumanoidAnimator : MonoBehaviour
{
    private Animator animator;

    void Start()
    {
        animator = GetComponent<Animator>();
    }

    // Call from ROS 2 messages
    public void SetWalkingState(bool isWalking)
    {
        animator.SetBool("IsWalking", isWalking);
    }

    public void SetSpeed(float speed)
    {
        animator.SetFloat("Speed", speed);
    }

    public void TriggerWave()
    {
        animator.SetTrigger("Wave");
    }
}
```

**Unity Animator Controller Setup:**
1. Create Animator Controller: Assets → Create → Animator Controller
2. Add states: Idle, Walking, Waving
3. Add parameters: IsWalking (bool), Speed (float), Wave (trigger)
4. Create transitions between states
5. Assign controller to humanoid model

### Key Takeaways (6 items)
1. **Unity excels at visualization and UX** - use it for demos, AR/VR, and user interfaces, not physics simulation
2. **ROS-TCP-Connector** bridges Unity and ROS 2 - bidirectional message passing over TCP/IP
3. **Coordinate system conversion** required: ROS (X-forward, Z-up) → Unity (Z-forward, Y-up)
4. **URDF Importer** simplifies bringing robot models into Unity with proper joint hierarchy
5. **Articulation Bodies** are Unity's equivalent of ROS joints - support position/velocity/torque control
6. **Gazebo + Unity** can work together - simulate in Gazebo, visualize in Unity (via ROS 2 topics)

### Exercises (4 questions)

#### Conceptual (1)
1. **Use Case Analysis**: For each scenario, recommend Gazebo, Unity, or both:
   - Testing autonomous navigation in a warehouse
   - Public demonstration of humanoid robot capabilities
   - VR teleoperation for remote surgery robot
   - Multi-robot swarm simulation (100+ robots)
   - Training a reinforcement learning agent

#### Coding (3)
2. **TurtleBot3 Visualization**:
   - Launch TurtleBot3 in Gazebo (use ROS 2 tutorials)
   - Create Unity scene to mirror TurtleBot3 position in real-time
   - Add UI buttons to control robot from Unity
   - Display camera feed from Gazebo camera in Unity

3. **Joint Control Dashboard**:
   - Create Unity UI with sliders for each robot joint
   - Publish joint commands to ROS 2 when sliders move
   - Display current joint positions from `/joint_states` feedback
   - Add min/max limits visualization

4. **AR Robot Overlay** (Advanced):
   - Use Unity AR Foundation package
   - Place virtual robot in real environment using AR
   - Control robot pose via ROS 2 messages
   - Add collision detection with real-world surfaces

### Next Steps
- **Module 3**: Learn NVIDIA Isaac Sim for GPU-accelerated simulation
- **Resources**:
  - [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
  - [ROS-TCP-Connector Tutorials](https://github.com/Unity-Technologies/ROS-TCP-Connector)
  - [Unity Learn: Robotics](https://learn.unity.com/course/unity-robotics)

---

## Task 2.3: Update Module 2 Intro

### File Location
`/my-website/docs/module-2/intro.md`

### Changes Required
Update the existing intro file to add:

1. **Duration header:**
```markdown
**Duration:** Weeks 6-7
```

2. **Weekly roadmap section:**
```markdown
## Weekly Roadmap

### Week 6: Gazebo Fundamentals
Learn to simulate robots in Gazebo for safe and rapid algorithm development:
- **URDF & SDF** - Robot and world description formats
- **Physics Engines** - ODE, Bullet, Simbody configuration
- **Sensor Simulation** - Camera, LIDAR, IMU plugins
- **ROS 2 Integration** - Launch robots and read sensor data

[→ Go to Week 6: Gazebo Fundamentals](/docs/module-2/week6-gazebo-fundamentals)

---

### Week 7: Unity Visualization
Build high-quality visualizations and user interfaces with Unity:
- **ROS-TCP-Connector** - Unity ↔ ROS 2 communication
- **Robot Visualization** - Import and animate robots from ROS 2
- **UI Development** - Control panels and dashboards
- **AR/VR Integration** - Immersive robot interaction

[→ Go to Week 7: Unity Visualization](/docs/module-2/week7-unity-visualization)

---
```

3. **What You'll Build section:**
```markdown
## What You'll Build

By the end of this module, you'll create:
- A custom robot model in URDF with sensors and actuators
- Gazebo simulation environment with obstacles and physics
- Unity visualization displaying robot state in real-time
- Control dashboard with UI buttons and sliders
- Integrated system: Gazebo (physics) + Unity (visualization)
```

### Preserve Existing Content
- ✅ Keep existing frontmatter (sidebar_position, title)
- ✅ Keep existing learning objectives
- ✅ Keep existing prerequisites
- ✅ Keep existing resources section

---

## Module 4: Conversational Robotics (Week 13)

### Overview
**Duration:** Week 13 (Capstone)
**Goal:** Integrate conversational AI (LLM + speech) with robot control
**Prerequisites:** Modules 1-3

---

## Task 2.4: Week 13 - Conversational AI Integration

### File Location
`/my-website/docs/module-4/week13-conversational-ai.md`

### Frontmatter
```yaml
---
sidebar_position: 2
title: Week 13 - Conversational AI Integration
---
```

### Learning Objectives (6 items)
1. Integrate Large Language Models (LLMs) for robotic task planning
2. Implement speech recognition using OpenAI Whisper
3. Parse natural language commands into structured robot actions
4. Handle multimodal input (voice + vision) for context awareness
5. Apply safety constraints and precondition checking
6. Build an end-to-end voice-controlled robot system

### Content Structure

#### 1. Conversational AI for Robotics
**Intuition:**
- Humans use language, robots use APIs
- LLMs bridge the gap: natural language → robot commands
- Voice interface makes robots accessible to non-experts

**Architecture:**
```
Voice Input (Microphone)
    ↓
Whisper (Speech → Text)
    ↓
LLM (Text → Task Plan)
    ↓
Action Parser (Plan → ROS 2 Actions)
    ↓
Robot Execution
    ↓
Feedback to User
```

**Use Cases:**
- Home assistance ("Bring me a glass of water")
- Warehouse logistics ("Move box from A5 to B3")
- Healthcare ("Check patient vitals in room 201")
- Manufacturing ("Inspect part #1234 for defects")

#### 2. Speech Recognition with Whisper
**Why Whisper?**
- State-of-the-art accuracy (OpenAI model)
- Multilingual support (99 languages)
- Robust to background noise
- Open-source and free

**Code Example 1: Install Whisper**
```bash
pip install openai-whisper
# Or for faster inference:
pip install whisper-jax
```

**Code Example 2: Basic Transcription**
```python
import whisper

# Load model (options: tiny, base, small, medium, large)
model = whisper.load_model("base")

# Transcribe audio file
result = model.transcribe("command.wav")
print(result["text"])
# Output: "Pick up the red cup and place it on the table"
```

**Code Example 3: Real-Time Microphone Input**
```python
import whisper
import pyaudio
import wave
import tempfile

class RealtimeWhisper:
    def __init__(self, model_size="base"):
        self.model = whisper.load_model(model_size)
        self.audio = pyaudio.PyAudio()

    def record_audio(self, duration=5):
        """Record audio from microphone"""
        CHUNK = 1024
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 16000

        stream = self.audio.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=RATE,
            input=True,
            frames_per_buffer=CHUNK
        )

        print(f"Recording for {duration} seconds...")
        frames = []

        for i in range(0, int(RATE / CHUNK * duration)):
            data = stream.read(CHUNK)
            frames.append(data)

        print("Recording finished.")
        stream.stop_stream()
        stream.close()

        # Save to temporary file
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
            wf = wave.open(f.name, 'wb')
            wf.setnchannels(CHANNELS)
            wf.setsampwidth(self.audio.get_sample_size(FORMAT))
            wf.setframerate(RATE)
            wf.writeframes(b''.join(frames))
            wf.close()
            return f.name

    def transcribe(self, audio_file=None, duration=5):
        """Transcribe audio (from file or microphone)"""
        if audio_file is None:
            audio_file = self.record_audio(duration)

        result = self.model.transcribe(audio_file)
        return result["text"]

# Usage
whisper_rt = RealtimeWhisper(model_size="base")

# Record and transcribe
command = whisper_rt.transcribe(duration=3)
print(f"You said: {command}")
```

#### 3. LLM Integration for Task Planning
**Why LLMs for Robotics?**
- Parse natural language → structured actions
- Handle ambiguity ("Pick up the cup" → which cup? use vision)
- Generate task sequences (multi-step plans)
- Adapt to context

**Code Example 4: GPT-4 Task Planner**
```python
import openai
import json

openai.api_key = "your-api-key-here"

def plan_task(command, context=""):
    """Convert natural language command to robot action plan"""

    system_prompt = """You are a robot task planner. Convert natural language commands
into structured JSON action sequences. Available actions:
- navigate(location: str)
- pick(object: str, location: str)
- place(object: str, location: str)
- inspect(object: str)
- wait(duration_sec: int)

Return a JSON list of actions. Example:
User: "Go to the kitchen and pick up the red cup"
Output: [
  {"action": "navigate", "location": "kitchen"},
  {"action": "pick", "object": "red_cup", "location": "kitchen_counter"}
]

Only output valid JSON. No explanations."""

    user_prompt = f"Command: {command}\nContext: {context}\n\nAction Plan:"

    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt}
        ],
        temperature=0.0  # Deterministic output
    )

    action_plan = json.loads(response.choices[0].message.content)
    return action_plan

# Example usage
command = "Pick up the red cup from the table and put it in the sink"
context = "Robot is in living room. Red cup detected on coffee table."

plan = plan_task(command, context)
print(json.dumps(plan, indent=2))

# Output:
# [
#   {"action": "navigate", "location": "coffee_table"},
#   {"action": "pick", "object": "red_cup", "location": "coffee_table"},
#   {"action": "navigate", "location": "kitchen"},
#   {"action": "place", "object": "red_cup", "location": "sink"}
# ]
```

**Code Example 5: Claude API Alternative (Anthropic)**
```python
import anthropic
import json

client = anthropic.Anthropic(api_key="your-api-key")

def plan_task_claude(command, context=""):
    """Use Claude for task planning"""

    system_prompt = """You are a robot task planner. Convert natural language commands
into JSON action sequences. Available actions: navigate, pick, place, inspect, wait.
Output only valid JSON."""

    message = client.messages.create(
        model="claude-3-5-sonnet-20241022",
        max_tokens=1024,
        system=system_prompt,
        messages=[
            {
                "role": "user",
                "content": f"Command: {command}\nContext: {context}\n\nAction Plan:"
            }
        ]
    )

    action_plan = json.loads(message.content[0].text)
    return action_plan

# Usage
plan = plan_task_claude("Bring me the blue book from the shelf")
print(plan)
```

#### 4. Intent Parsing and Action Schema
**Action Schema Design:**
```python
from dataclasses import dataclass
from typing import List, Optional
from enum import Enum

class ActionType(Enum):
    NAVIGATE = "navigate"
    PICK = "pick"
    PLACE = "place"
    INSPECT = "inspect"
    WAIT = "wait"
    SPEAK = "speak"

@dataclass
class RobotAction:
    action_type: ActionType
    object: Optional[str] = None
    location: Optional[str] = None
    duration_sec: Optional[int] = None
    message: Optional[str] = None

    def to_dict(self):
        return {
            "action": self.action_type.value,
            "object": self.object,
            "location": self.location,
            "duration_sec": self.duration_sec,
            "message": self.message
        }

@dataclass
class TaskPlan:
    command: str
    actions: List[RobotAction]
    confidence: float

    def validate(self):
        """Check if plan is executable"""
        for action in self.actions:
            if action.action_type == ActionType.NAVIGATE and not action.location:
                return False, "Navigate action missing location"
            if action.action_type in [ActionType.PICK, ActionType.PLACE]:
                if not action.object or not action.location:
                    return False, f"{action.action_type.value} action missing object/location"
        return True, "Plan is valid"
```

**Code Example 6: Intent Parser**
```python
import re
import json

class IntentParser:
    def __init__(self):
        self.patterns = {
            'navigate': r'go to|move to|navigate to',
            'pick': r'pick up|grab|take|get',
            'place': r'place|put|set down',
            'inspect': r'inspect|check|examine|look at'
        }

    def parse_simple(self, command):
        """Simple regex-based parsing (no LLM)"""
        command_lower = command.lower()

        # Detect action type
        action_type = None
        for action, pattern in self.patterns.items():
            if re.search(pattern, command_lower):
                action_type = action
                break

        if not action_type:
            return None

        # Extract object and location (simple approach)
        # For production, use NER (Named Entity Recognition)
        words = command.split()
        object_name = None
        location = None

        # Find color + noun (e.g., "red cup")
        colors = ['red', 'blue', 'green', 'yellow', 'black', 'white']
        for i, word in enumerate(words):
            if word.lower() in colors and i+1 < len(words):
                object_name = f"{word}_{words[i+1]}"
                break

        # Find location (after "from", "to", "on", "in")
        prepositions = ['from', 'to', 'on', 'in', 'at']
        for i, word in enumerate(words):
            if word.lower() in prepositions and i+1 < len(words):
                location = words[i+1]
                break

        return RobotAction(
            action_type=ActionType(action_type),
            object=object_name,
            location=location
        )

    def parse_with_llm(self, command, context=""):
        """Use LLM for robust parsing"""
        plan = plan_task(command, context)
        actions = []
        for action_dict in plan:
            action = RobotAction(
                action_type=ActionType(action_dict['action']),
                object=action_dict.get('object'),
                location=action_dict.get('location'),
                duration_sec=action_dict.get('duration_sec'),
                message=action_dict.get('message')
            )
            actions.append(action)
        return TaskPlan(command=command, actions=actions, confidence=0.9)

# Usage
parser = IntentParser()

# Simple parsing
action1 = parser.parse_simple("Pick up the red cup from the table")
print(action1)

# LLM parsing
plan = parser.parse_with_llm("Go to the kitchen and bring me the blue mug")
print(f"Plan: {[a.to_dict() for a in plan.actions]}")
```

#### 5. ROS 2 Action Integration
**Code Example 7: Execute Actions via ROS 2**
```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped

class RobotActionExecutor(Node):
    def __init__(self):
        super().__init__('robot_action_executor')
        # Initialize action clients (example)
        # Real implementations would use specific action types

    def execute_navigate(self, location):
        """Navigate to location"""
        self.get_logger().info(f"Navigating to {location}")
        # Use Nav2 action client
        # goal = NavigateToPose.Goal()
        # goal.pose.header.frame_id = "map"
        # goal.pose.pose.position.x = ...
        # Send goal and wait for result
        return True

    def execute_pick(self, object_name, location):
        """Pick object at location"""
        self.get_logger().info(f"Picking {object_name} at {location}")
        # Use MoveIt2 or custom grasp action
        # 1. Move arm to pre-grasp pose
        # 2. Open gripper
        # 3. Move to grasp pose
        # 4. Close gripper
        # 5. Lift object
        return True

    def execute_place(self, object_name, location):
        """Place object at location"""
        self.get_logger().info(f"Placing {object_name} at {location}")
        # 1. Move to pre-place pose
        # 2. Lower to place pose
        # 3. Open gripper
        # 4. Retract arm
        return True

    def execute_plan(self, task_plan):
        """Execute full task plan"""
        for action in task_plan.actions:
            self.get_logger().info(f"Executing: {action.action_type.value}")

            if action.action_type == ActionType.NAVIGATE:
                success = self.execute_navigate(action.location)
            elif action.action_type == ActionType.PICK:
                success = self.execute_pick(action.object, action.location)
            elif action.action_type == ActionType.PLACE:
                success = self.execute_place(action.object, action.location)
            elif action.action_type == ActionType.WAIT:
                import time
                time.sleep(action.duration_sec)
                success = True
            else:
                self.get_logger().warn(f"Unknown action: {action.action_type}")
                success = False

            if not success:
                self.get_logger().error(f"Action failed: {action.action_type.value}")
                return False

        self.get_logger().info("Plan executed successfully!")
        return True

# Usage
def main():
    rclpy.init()
    executor = RobotActionExecutor()

    # Example plan
    plan = TaskPlan(
        command="Pick up red cup from table",
        actions=[
            RobotAction(ActionType.NAVIGATE, location="table"),
            RobotAction(ActionType.PICK, object="red_cup", location="table"),
        ],
        confidence=0.9
    )

    executor.execute_plan(plan)
    executor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 6. Complete Voice-to-Action Pipeline
**Code Example 8: End-to-End System**
```python
import rclpy
from rclpy.node import Node
import whisper
import openai

class VoiceControlledRobot(Node):
    def __init__(self):
        super().__init__('voice_controlled_robot')

        # Initialize Whisper
        self.whisper_model = whisper.load_model("base")
        self.get_logger().info("Whisper model loaded")

        # Initialize LLM
        openai.api_key = "your-api-key"

        # Initialize robot action executor
        self.action_executor = RobotActionExecutor()

        # Initialize intent parser
        self.intent_parser = IntentParser()

    def listen_and_execute(self):
        """Main loop: Listen → Parse → Execute"""
        print("\n=== Voice-Controlled Robot Ready ===")
        print("Press Enter to start recording (3 seconds)...")
        input()

        # 1. Speech Recognition
        print("🎤 Listening...")
        whisper_rt = RealtimeWhisper(model_size="base")
        command = whisper_rt.transcribe(duration=3)
        print(f"📝 Transcribed: {command}")

        # 2. Intent Parsing with LLM
        print("🤖 Planning task...")
        task_plan = self.intent_parser.parse_with_llm(command)

        # 3. Validate plan
        valid, message = task_plan.validate()
        if not valid:
            print(f"❌ Invalid plan: {message}")
            return

        print(f"✅ Plan created:")
        for i, action in enumerate(task_plan.actions):
            print(f"  {i+1}. {action.action_type.value} - {action.object} @ {action.location}")

        # 4. Execute plan
        print("🚀 Executing...")
        success = self.action_executor.execute_plan(task_plan)

        if success:
            print("✅ Task completed successfully!")
        else:
            print("❌ Task failed")

def main():
    rclpy.init()
    robot = VoiceControlledRobot()

    try:
        while rclpy.ok():
            robot.listen_and_execute()
    except KeyboardInterrupt:
        pass

    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 7. Multimodal Integration (Voice + Vision)
**Scenario:** "Pick up that cup" (requires vision to identify "that")

**Code Example 9: Vision-Based Object Grounding**
```python
import cv2
from ultralytics import YOLO

class ObjectGrounder:
    def __init__(self):
        self.yolo_model = YOLO("yolov8n.pt")  # Nano model

    def detect_objects(self, image_path):
        """Detect objects in image"""
        results = self.yolo_model(image_path)
        objects = []

        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                class_name = result.names[class_id]
                confidence = float(box.conf[0])
                bbox = box.xyxy[0].tolist()  # [x1, y1, x2, y2]

                objects.append({
                    'class': class_name,
                    'confidence': confidence,
                    'bbox': bbox
                })

        return objects

    def resolve_reference(self, command, detected_objects):
        """Resolve "that cup" using vision + spatial reasoning"""
        # Simple approach: find object closest to center
        # Advanced: use pointing gesture detection or gaze tracking

        if "that" in command.lower():
            # Find object type from command
            object_types = ['cup', 'bottle', 'book', 'phone']
            target_type = None
            for obj_type in object_types:
                if obj_type in command.lower():
                    target_type = obj_type
                    break

            if target_type:
                # Find matching objects
                matches = [obj for obj in detected_objects
                          if target_type in obj['class'].lower()]

                if matches:
                    # Return object closest to image center (simple heuristic)
                    center_x, center_y = 640/2, 480/2  # Image dimensions
                    closest = min(matches, key=lambda obj:
                        ((obj['bbox'][0] + obj['bbox'][2])/2 - center_x)**2 +
                        ((obj['bbox'][1] + obj['bbox'][3])/2 - center_y)**2
                    )
                    return closest

        return None

# Integration example
grounder = ObjectGrounder()

# Capture camera image (from ROS 2 topic)
# cv_image = ... (from /camera/image_raw)

# Detect objects
objects = grounder.detect_objects("scene.jpg")
print(f"Detected: {[obj['class'] for obj in objects]}")

# Resolve "that cup"
command = "Pick up that cup"
target = grounder.resolve_reference(command, objects)
if target:
    print(f"Target identified: {target['class']} at {target['bbox']}")
```

#### 8. Safety Constraints
**Code Example 10: Safety Validator**
```python
class SafetyValidator:
    def __init__(self):
        self.workspace_bounds = {
            'x': (-2.0, 2.0),  # meters
            'y': (-2.0, 2.0),
            'z': (0.0, 2.0)
        }
        self.forbidden_zones = [
            {'x': (0.9, 1.1), 'y': (0.9, 1.1)},  # Human standing area
        ]
        self.max_velocity = 0.5  # m/s
        self.dangerous_objects = ['knife', 'glass', 'hot']

    def check_action_safety(self, action):
        """Validate action before execution"""
        # Check 1: Workspace bounds
        if action.location:
            # Parse location to coordinates (simplified)
            # In production, use semantic mapping
            pass

        # Check 2: Dangerous objects
        if action.object:
            for dangerous in self.dangerous_objects:
                if dangerous in action.object.lower():
                    return False, f"Cannot manipulate dangerous object: {action.object}"

        # Check 3: Speed limits (for navigate actions)
        if action.action_type == ActionType.NAVIGATE:
            # Check planned trajectory doesn't exceed velocity limits
            pass

        return True, "Action is safe"

    def check_plan_safety(self, task_plan):
        """Validate entire plan"""
        for i, action in enumerate(task_plan.actions):
            safe, message = self.check_action_safety(action)
            if not safe:
                return False, f"Action {i+1} unsafe: {message}"
        return True, "Plan is safe"

    def require_confirmation(self, task_plan):
        """Check if plan requires human confirmation"""
        critical_keywords = ['delete', 'throw', 'break', 'dangerous']

        for word in critical_keywords:
            if word in task_plan.command.lower():
                return True

        return False

# Usage
validator = SafetyValidator()

plan = TaskPlan(
    command="Pick up the knife from the counter",
    actions=[RobotAction(ActionType.PICK, object="knife", location="counter")],
    confidence=0.9
)

safe, message = validator.check_plan_safety(plan)
if not safe:
    print(f"❌ Unsafe plan: {message}")
else:
    if validator.require_confirmation(plan):
        print("⚠️  This action requires confirmation. Proceed? (y/n)")
        # Wait for user input
```

### Key Takeaways (8 items)
1. **Conversational AI makes robots accessible** - natural language interface removes programming barrier
2. **Whisper provides state-of-the-art speech recognition** - robust, multilingual, and open-source
3. **LLMs excel at task planning** - convert natural language to structured action sequences
4. **Action schemas bridge LLM output and robot APIs** - define clear contract for executable actions
5. **Multimodal input (voice + vision) resolves ambiguity** - "that cup" requires camera input
6. **Safety validation is critical** - check workspace bounds, dangerous objects, and require confirmation
7. **Context awareness improves accuracy** - include robot state and environment in LLM prompts
8. **Graceful degradation handles failures** - retry with clarification questions, not silent failures

### Exercises (5 questions)

#### Conceptual (2)
1. **Safety Analysis**: List 5 potential safety risks in voice-controlled robots. For each risk, propose a mitigation strategy (precondition check, confirmation dialog, etc.).

2. **Ambiguity Resolution**: Given the command "Put the box on the table", what information is ambiguous? How would you use:
   - Vision (camera)
   - Dialogue (clarifying questions)
   - Context (robot state, past actions)
   to resolve ambiguity?

#### Coding (3)
3. **Whisper + ROS 2 Node**: Create a ROS 2 node that:
   - Continuously listens for voice commands (use wake word "robot")
   - Transcribes with Whisper
   - Publishes transcribed text to `/voice_command` topic
   - Provide start/stop service for recording

4. **LLM Task Planner with Few-Shot Examples**:
   - Improve the task planner with 5 few-shot examples in the system prompt
   - Add error handling for invalid JSON responses
   - Include confidence scores for each action
   - Test with 10 diverse commands

5. **Complete Voice-Controlled TurtleBot** (Capstone):
   - Set up TurtleBot3 in Gazebo
   - Integrate Whisper for voice commands
   - Use LLM to parse: "Go forward 2 meters", "Turn left 90 degrees", etc.
   - Execute via `/cmd_vel` topic
   - Add safety: stop if obstacle detected (LIDAR)
   - Demo video required!

### Next Steps
- **Capstone Project**: Integrate all modules (ROS 2, simulation, kinematics, conversational AI)
- **Resources**:
  - [OpenAI Whisper GitHub](https://github.com/openai/whisper)
  - [LangChain for Robotics](https://python.langchain.com/docs/use_cases/robotics)
  - [Prompt Engineering Guide](https://www.promptingguide.ai/)

---

## Task 2.5: Update Module 4 Intro

### File Location
`/my-website/docs/module-4/intro.md`

### Changes Required
Update the existing intro file to add:

1. **Duration header:**
```markdown
**Duration:** Week 13 (Capstone)
```

2. **Weekly roadmap section:**
```markdown
## Weekly Roadmap

### Week 13: Conversational AI Integration
Bring it all together - build voice-controlled robots with LLM-powered task planning:
- **Speech Recognition** - OpenAI Whisper for voice commands
- **LLM Task Planning** - GPT-4/Claude for natural language → robot actions
- **Multimodal Input** - Combine voice + vision for context awareness
- **Safety Constraints** - Precondition checking and user confirmation
- **End-to-End Pipeline** - Voice → Intent → Action → Execution

[→ Go to Week 13: Conversational AI Integration](/docs/module-4/week13-conversational-ai)

---
```

3. **What You'll Build section:**
```markdown
## What You'll Build

By the end of this module, you'll create:
- Voice-controlled robot system using Whisper + LLM
- Task planner that converts natural language to robot actions
- Multimodal integration (voice commands + camera vision)
- Safety validator with precondition checks
- Complete demo: "Hey robot, pick up the red cup and place it on the table"
```

### Preserve Existing Content
- ✅ Keep existing frontmatter (sidebar_position, title)
- ✅ Keep existing learning objectives
- ✅ Keep existing prerequisites
- ✅ Keep existing resources section

---

## Phase 2 File Summary

### Files to Create (3)
1. `/my-website/docs/module-2/week6-gazebo-fundamentals.md` (NEW)
2. `/my-website/docs/module-2/week7-unity-visualization.md` (NEW)
3. `/my-website/docs/module-4/week13-conversational-ai.md` (NEW)

### Files to Update (2)
4. `/my-website/docs/module-2/intro.md` (UPDATE - add weekly roadmap)
5. `/my-website/docs/module-4/intro.md` (UPDATE - add weekly roadmap)

**Total:** 5 files (3 new + 2 updates)

---

## Validation Checkpoint 2

### After Phase 2 Completion, Validate:

1. **Build Test**
```bash
cd my-website
npm run build
```
- [ ] Build succeeds without errors
- [ ] No broken links reported
- [ ] Both locales (en, ur) build correctly

2. **Content Validation**
- [ ] Module 2: Weeks 6-7 present + intro updated
- [ ] Module 4: Week 13 present + intro updated
- [ ] All internal links functional
- [ ] Code examples have proper syntax highlighting

3. **Quality Check**
- [ ] Learning objectives (5-6 per week)
- [ ] Code examples (8-10+ per week, all runnable)
- [ ] Key takeaways (6-8 per week)
- [ ] Exercises (4-5 per week)
- [ ] Consistent style with Phase 1

4. **Technical Accuracy**
- [ ] URDF examples valid XML
- [ ] Gazebo commands correct
- [ ] Unity C# code compiles
- [ ] Whisper/LLM code examples functional
- [ ] ROS 2 integration code accurate

**STOP POINT:** Review Phase 2 completion before proceeding to Phase 3

---

## Success Criteria Summary

### Content Requirements
- ✅ 5 files created/updated
- ✅ 3 new comprehensive weeks (6, 7, 13)
- ✅ 2 intro files updated with roadmaps
- ✅ 20+ code examples per week
- ✅ Self-contained, beginner-friendly content

### Technical Requirements
- ✅ All code examples runnable (Bash, Python, C#, XML)
- ✅ Proper frontmatter (sidebar_position, title)
- ✅ Internal links use Docusaurus format
- ✅ Markdown syntax valid
- ✅ Build passes without errors

### Pedagogical Requirements
- ✅ Intuition before complexity
- ✅ Progressive difficulty
- ✅ Real-world examples
- ✅ Hands-on exercises
- ✅ Clear next steps

---

## Execution Notes

### For Implementation Agent:
1. **Execute tasks sequentially** (2.1 → 2.2 → 2.3 → 2.4 → 2.5)
2. **Follow Phase 1 patterns** for consistency
3. **Use Module 1/5 as style reference**
4. **Create PHRs for each task**
5. **Run validation checkpoint after all tasks**

### Quality Standards:
- **Code**: All examples must be runnable (tested or clearly pseudo-code)
- **Math**: Keep equations simple, focus on intuition
- **Style**: Match Module 1 and Module 5 voice and structure
- **Length**: 800-1200 lines per week chapter
- **Exercises**: Mix conceptual (1-2) + coding (3-4)

---

**Specification Status:** ✅ READY FOR IMPLEMENTATION
**Next Step:** Execute Phase 2, Task 2.1 (Week 6: Gazebo Fundamentals)
