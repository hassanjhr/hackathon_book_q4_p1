# Phase 3 Specification: NVIDIA Isaac Sim & AI for Robotics

**Date:** 2025-12-17
**Phase:** 3 (Module 3: NVIDIA Isaac Platform)
**Priority:** MEDIUM
**Duration:** 3 weeks (Weeks 8-10)

---

## Overview

**Goal:** Complete Module 3 content covering NVIDIA Isaac Sim, Isaac ROS, and Reinforcement Learning fundamentals for robotics.

**Scope:**
- Module 3 (AI-Robot Brain): Weeks 8-10
- Update Module 3 intro with weekly roadmap
- Content-only changes (no deployment)

**Constraints:**
- Python-first approach
- Beginner → intermediate progression
- Runnable examples (where possible without GPU)
- Conceptual + practical balance
- No infrastructure changes

---

## Files to Create/Modify

### Summary Table

| # | File | Type | Lines | Status |
|---|------|------|-------|--------|
| 1 | `module-3/week8-isaac-sim-fundamentals.md` | NEW | ~900 | Pending |
| 2 | `module-3/week9-isaac-ros-integration.md` | NEW | ~850 | Pending |
| 3 | `module-3/week10-reinforcement-learning.md` | NEW | ~800 | Pending |
| 4 | `module-3/intro.md` | UPDATE | - | Pending |

**Total:** 4 files, ~2,550 new lines

---

## Week 8: Isaac Sim Fundamentals

### File: `week8-isaac-sim-fundamentals.md`
**Location:** `/my-website/docs/module-3/week8-isaac-sim-fundamentals.md`

**Frontmatter:**
```yaml
---
sidebar_position: 2
title: Week 8 - Isaac Sim Fundamentals
---
```

---

### Learning Objectives (6 items)

- Understand NVIDIA Isaac Sim architecture and capabilities
- Install and configure Isaac Sim (Omniverse platform)
- Generate synthetic training data for vision models
- Simulate cameras, LIDAR, and sensors with realistic physics
- Use Python API for programmatic scene control
- Export datasets for machine learning pipelines

---

### Content Outline

#### 1. **Why NVIDIA Isaac Sim?**

**Conceptual:**
- Photorealistic rendering (RTX ray tracing)
- Accurate physics simulation (PhysX)
- Synthetic data generation at scale
- Sim-to-real transfer advantages
- Use cases: training perception models, robot testing, digital twins

**Comparison Table: Gazebo vs Isaac Sim**

| Feature | Gazebo | Isaac Sim |
|---------|--------|-----------|
| **Graphics** | Good (OpenGL) | Photorealistic (RTX) |
| **Physics** | ODE/Bullet | NVIDIA PhysX (GPU) |
| **Sensors** | Basic | Realistic (noise, artifacts) |
| **AI/ML** | Limited | Native integration |
| **Performance** | CPU | GPU-accelerated |
| **Use Case** | Algorithm dev | AI training + sim-to-real |

**When to use each:**
- **Gazebo**: Algorithm prototyping, ROS 2 testing, CPU-only systems
- **Isaac Sim**: AI model training, photorealistic visualization, GPU available

---

#### 2. **Isaac Sim Architecture**

**Components:**
1. **Omniverse Kit**: Foundation (USD scene graph, rendering)
2. **Isaac Sim**: Robotics-specific features
3. **PhysX**: GPU-accelerated physics
4. **RTX Renderer**: Ray tracing for photorealism
5. **Python API**: Automation and scripting
6. **ROS 2 Bridge**: Integration with ROS ecosystem

**Diagram (ASCII):**
```
┌─────────────────────────────────────┐
│     Isaac Sim Application           │
├─────────────────────────────────────┤
│  Python API  │  GUI  │  ROS 2 Bridge│
├──────────────┴───────┴──────────────┤
│         PhysX (Physics)              │
│         RTX (Rendering)              │
├─────────────────────────────────────┤
│    Omniverse Kit (USD Runtime)      │
└─────────────────────────────────────┘
```

---

#### 3. **Installation and Setup**

**Prerequisites:**
- NVIDIA GPU: RTX 3060+ (12GB+ VRAM)
- Ubuntu 20.04/22.04 or Windows 10/11
- NVIDIA Driver: 525+
- Python 3.10

**Installation Steps:**

```bash
# 1. Install Omniverse Launcher
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage

# 2. Through Omniverse Launcher:
# - Install "Nucleus" (asset server)
# - Install "Isaac Sim 2023.1.1" or later
# - Launch Isaac Sim

# 3. Verify installation (Python standalone)
~/.local/share/ov/pkg/isaac_sim-*/python.sh --version
```

**First Launch:**
```python
# Basic Isaac Sim Python script
from isaacsim import SimulationApp

# Initialize simulation
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

# Create world
world = World()
world.scene.add_default_ground_plane()

# Add a cube
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="my_cube",
        position=[0, 0, 1.0],
        size=0.5,
        color=[1.0, 0.0, 0.0]  # Red
    )
)

# Run simulation
for i in range(100):
    world.step(render=True)

simulation_app.close()
```

**Output:** Opens Isaac Sim GUI with a red cube falling onto a ground plane.

---

#### 4. **Synthetic Data Generation**

**Why Synthetic Data?**
- **Cost**: Free vs expensive real-world labeling
- **Scale**: Generate millions of images automatically
- **Diversity**: Randomize lighting, textures, objects
- **Perfect labels**: Ground truth for segmentation, depth, pose

**Use Cases:**
- Object detection model training
- Semantic segmentation
- Depth estimation
- Pose estimation

**Example: Camera Setup and Data Capture**

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.sensor import Camera
import omni.replicator.core as rep
import numpy as np

# Create world
world = World()
world.scene.add_default_ground_plane()

# Load a robot model (example: Franka Panda)
add_reference_to_stage(
    usd_path="/Isaac/Robots/Franka/franka_alt_fingers.usd",
    prim_path="/World/Franka"
)

# Add camera
camera = Camera(
    prim_path="/World/Camera",
    position=np.array([2.0, 2.0, 1.5]),
    frequency=30,  # 30 Hz
    resolution=(1280, 720)
)

# Initialize and play
world.reset()

# Capture 10 frames
for i in range(10):
    world.step(render=True)

    # Get camera data
    rgb_data = camera.get_rgba()  # Shape: (720, 1280, 4)
    depth_data = camera.get_depth()  # Shape: (720, 1280)

    print(f"Frame {i}: RGB shape={rgb_data.shape}, Depth shape={depth_data.shape}")

simulation_app.close()
```

---

#### 5. **Domain Randomization**

**Concept:** Randomize scene parameters to improve sim-to-real transfer.

**Randomization Strategies:**
- **Lighting**: Color temperature, intensity, position
- **Textures**: Materials, colors
- **Object poses**: Position, orientation
- **Camera**: Position, focal length, noise
- **Physics**: Friction, mass, damping

**Code Example: Randomized Scene**

```python
import omni.replicator.core as rep
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})
from omni.isaac.core import World

world = World()
world.scene.add_default_ground_plane()

# Define randomization
with rep.new_layer():
    # Create 10 cubes with random positions/colors
    cubes = rep.create.cube(
        semantics=[("class", "cube")],
        count=10,
        position=rep.distribution.uniform((-2, -2, 0.5), (2, 2, 2)),
        scale=rep.distribution.uniform(0.1, 0.5),
        color=rep.distribution.uniform((0, 0, 0), (1, 1, 1))
    )

    # Randomize lighting
    lights = rep.create.light(
        light_type="Sphere",
        intensity=rep.distribution.uniform(1000, 5000),
        position=rep.distribution.uniform((-5, -5, 3), (5, 5, 8)),
        count=3
    )

    # Camera
    camera = rep.create.camera(position=(3, 3, 2), look_at=(0, 0, 0))

    # Render settings
    render_product = rep.create.render_product(camera, (512, 512))

    # Writer (save images)
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir="./synthetic_data", rgb=True, bounding_box_2d_tight=True)
    writer.attach([render_product])

# Generate 100 randomized scenes
with rep.trigger.on_frame(num_frames=100):
    rep.randomizer.scatter_2d(cubes, surface_prims="/World/defaultGroundPlane")

rep.orchestrator.run()

simulation_app.close()
```

**Output:** 100 images saved to `./synthetic_data/` with randomized cube positions, colors, and lighting.

---

#### 6. **Sensor Simulation**

**Simulated Sensors:**
- **RGB Camera**: Photorealistic images
- **Depth Camera**: Accurate depth maps
- **Semantic Segmentation**: Per-pixel class labels
- **LIDAR**: 3D point clouds
- **IMU**: Accelerometer + gyroscope data
- **Contact Sensors**: Force/torque feedback

**Example: LIDAR Simulation**

```python
from omni.isaac.range_sensor import _range_sensor
import numpy as np

# Create LIDAR
lidar_config = _range_sensor.acquire_lidar_sensor_interface()

# Configure rotating LIDAR (360°, 32 beams)
lidar_path = "/World/Lidar"
lidar_config.set_rotation_frequency(lidar_path, 20.0)  # 20 Hz
lidar_config.set_horizontal_fov(lidar_path, 360.0)  # Degrees
lidar_config.set_horizontal_resolution(lidar_path, 0.4)  # Degrees
lidar_config.set_num_rows(lidar_path, 32)  # 32 vertical beams

# Run simulation and get point cloud
world.step(render=True)

# Get LIDAR data
point_cloud = lidar_config.get_point_cloud_data(lidar_path)  # Shape: (N, 3)
print(f"Point cloud: {point_cloud.shape} points")

# Visualize (convert to numpy)
points = np.array(point_cloud)
# ... use Open3D or similar for visualization
```

---

#### 7. **Python API: Programmatic Control**

**Common Tasks:**
1. Spawn/delete objects
2. Apply forces
3. Get/set object properties
4. Control robot joints
5. Trigger events

**Example: Procedural Scene Generation**

```python
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.objects import DynamicCuboid
import random

world = World()
world.scene.add_default_ground_plane()

# Generate 50 random cubes
for i in range(50):
    x = random.uniform(-5, 5)
    y = random.uniform(-5, 5)
    z = random.uniform(1, 3)
    size = random.uniform(0.1, 0.5)

    cube = DynamicCuboid(
        prim_path=f"/World/Cube_{i}",
        position=[x, y, z],
        size=size,
        color=[random.random(), random.random(), random.random()]
    )
    world.scene.add(cube)

# Simulate physics
for step in range(500):
    world.step(render=True)

simulation_app.close()
```

---

#### 8. **Exporting Data for ML Pipelines**

**Data Formats:**
- **Images**: PNG, JPEG
- **Annotations**: COCO JSON, YOLO format, Pascal VOC
- **Depth**: NPY, PNG (16-bit)
- **Point Clouds**: PCD, PLY

**Example: COCO Format Export**

```python
import omni.replicator.core as rep

# Create writer for COCO format
writer = rep.WriterRegistry.get("COCOWriter")
writer.initialize(
    output_dir="./coco_dataset",
    rgb=True,
    bounding_box_2d_tight=True,
    semantic_segmentation=True
)

# Attach to render product
writer.attach([render_product])

# Run and export
rep.orchestrator.run()
```

**Output Directory Structure:**
```
coco_dataset/
├── annotations/
│   └── instances.json
├── images/
│   ├── 00000.png
│   ├── 00001.png
│   └── ...
└── masks/
    ├── 00000.png
    ├── 00001.png
    └── ...
```

---

### Key Takeaways (8 items)

1. **Isaac Sim enables photorealistic simulation** with GPU-accelerated physics and RTX rendering
2. **Synthetic data generation solves the data scarcity problem** for training vision models
3. **Domain randomization** (lighting, textures, poses) improves sim-to-real transfer
4. **Python API provides full programmatic control** for automation and integration
5. **Sensor simulation is realistic** - cameras, LIDAR, IMU with noise models
6. **Data export supports standard ML formats** (COCO, YOLO, Pascal VOC)
7. **Hardware requirements are significant** - RTX 3060+ with 12GB+ VRAM
8. **Isaac Sim integrates with ROS 2** for robot testing and development

---

### Exercises (5 questions)

#### Conceptual (2)
1. **Sim-to-real gap**: Why does a model trained only on synthetic data often fail on real images? List 3 domain randomization strategies that help bridge this gap.

2. **Gazebo vs Isaac Sim**: You need to test a navigation algorithm AND train an object detection model. Which simulator(s) would you use for each task, and why?

#### Coding (3)
3. **Basic scene**: Write an Isaac Sim Python script that creates 5 spheres at random positions, simulates them falling with gravity, and saves RGB + depth images every 10 frames.

4. **Domain randomization**: Extend Exercise 3 to randomize sphere colors, lighting intensity, and camera position for each frame.

5. **LIDAR data**: Create a scene with obstacles (cubes, cylinders) and a rotating LIDAR sensor. Extract the point cloud and print the minimum distance to any obstacle.

---

### Further Resources

**Official Documentation:**
- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [Replicator Tutorials](https://docs.omniverse.nvidia.com/prod_extensions/prod_extensions/ext_replicator.html)
- [Python API Reference](https://docs.omniverse.nvidia.com/py/isaacsim/index.html)

**Tutorials:**
- NVIDIA Isaac Sim YouTube channel
- "Synthetic Data Generation for Robotics" (NVIDIA blog)
- Isaac Sim Examples (GitHub)

**Next:** [Week 9: Isaac ROS Integration](./week9-isaac-ros-integration)

---

## Week 9: Isaac ROS Integration

### File: `week9-isaac-ros-integration.md`
**Location:** `/my-website/docs/module-3/week9-isaac-ros-integration.md`

**Frontmatter:**
```yaml
---
sidebar_position: 3
title: Week 9 - Isaac ROS Integration
---
```

---

### Learning Objectives (6 items)

- Understand Isaac ROS architecture and GEM (GPU-Accelerated Extensible Modules)
- Install and configure Isaac ROS packages
- Build GPU-accelerated perception pipelines
- Integrate Isaac ROS with custom ROS 2 nodes
- Perform real-time object detection with NVIDIA models
- Understand VSLAM (Visual Simultaneous Localization and Mapping)

---

### Content Outline

#### 1. **What is Isaac ROS?**

**Definition:** Isaac ROS is a collection of **GPU-accelerated ROS 2 packages** for robotics perception, navigation, and manipulation.

**Key Features:**
- **GPU acceleration**: 10-100x faster than CPU
- **NVIDIA hardware-specific**: Jetson, RTX GPUs
- **Production-ready**: Optimized for real-time performance
- **Modular**: Use individual packages or full pipelines

**GEMs (GPU-Accelerated Extensible Modules):**
- Reusable building blocks for perception
- Optimized kernels (CUDA, TensorRT)
- Standard ROS 2 interfaces

**Architecture:**
```
ROS 2 Node (Python/C++)
    ↓
Isaac ROS GEM (GPU-accelerated)
    ↓ (TensorRT, CUDA)
NVIDIA Hardware (Jetson, RTX)
```

---

#### 2. **Isaac ROS Packages Overview**

**Key Packages:**

| Package | Purpose | Use Case |
|---------|---------|----------|
| `isaac_ros_visual_slam` | cuVSLAM | Localization & mapping |
| `isaac_ros_dnn_inference` | TensorRT inference | Object detection, segmentation |
| `isaac_ros_image_proc` | Image processing | Debayering, rectification |
| `isaac_ros_apriltag` | AprilTag detection | Fiducial markers |
| `isaac_ros_depth_image_proc` | Depth processing | Point clouds, normals |
| `isaac_ros_nvblox` | 3D reconstruction | Occupancy mapping |

**Comparison: Traditional ROS 2 vs Isaac ROS**

| Task | ROS 2 (CPU) | Isaac ROS (GPU) |
|------|-------------|-----------------|
| Image rectification | 30 FPS | 200+ FPS |
| Object detection (YOLOv8) | 5 FPS | 60+ FPS |
| VSLAM | 10 FPS | 100+ FPS |
| Point cloud processing | 5 FPS | 50+ FPS |

---

#### 3. **Installation**

**Prerequisites:**
- Ubuntu 22.04
- ROS 2 Humble
- NVIDIA GPU (Jetson or RTX)
- CUDA 12.0+
- TensorRT 8.6+

**Install Isaac ROS (Docker-based):**

```bash
# Install Docker
curl https://get.docker.com | sh
sudo systemctl --now enable docker

# Add user to docker group
sudo usermod -aG docker $USER
newgrp docker

# Clone Isaac ROS common
mkdir -p ~/workspaces/isaac_ros-dev/src
cd ~/workspaces/isaac_ros-dev/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# Build Docker image
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common
./scripts/run_dev.sh

# Inside Docker container, install packages
cd /workspaces/isaac_ros-dev
colcon build --symlink-install

source install/setup.bash
```

**Verify Installation:**
```bash
ros2 pkg list | grep isaac_ros
# Should show: isaac_ros_visual_slam, isaac_ros_dnn_inference, etc.
```

---

#### 4. **Example: Object Detection with DOPE**

**DOPE (Deep Object Pose Estimation):** Detects 3D pose of known objects.

**Install DOPE:**
```bash
cd ~/workspaces/isaac_ros-dev/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation.git

cd ~/workspaces/isaac_ros-dev
colcon build --packages-up-to isaac_ros_dope
source install/setup.bash
```

**Launch DOPE Node:**
```bash
# Terminal 1: Start camera (or use rosbag)
ros2 run usb_cam usb_cam_node_exe

# Terminal 2: Launch DOPE
ros2 launch isaac_ros_dope isaac_ros_dope_tensor_rt.launch.py \
    model_file_path:=/workspaces/isaac_ros-dev/models/ketchup.onnx \
    engine_file_path:=/workspaces/isaac_ros-dev/models/ketchup.plan

# Terminal 3: Visualize
rviz2
```

**Expected Output:**
- Detected object pose published to `/dope/pose_array`
- 3D bounding box overlay on camera image

**Custom Python Subscriber:**
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray

class DopeListener(Node):
    def __init__(self):
        super().__init__('dope_listener')
        self.subscription = self.create_subscription(
            PoseArray,
            '/dope/pose_array',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        for i, pose in enumerate(msg.poses):
            pos = pose.position
            self.get_logger().info(
                f'Object {i}: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = DopeListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

#### 5. **Visual SLAM with cuVSLAM**

**cuVSLAM:** GPU-accelerated Visual SLAM for real-time localization and mapping.

**Features:**
- Stereo or monocular camera
- IMU fusion
- Loop closure detection
- Real-time performance (100+ FPS)

**Launch cuVSLAM:**
```bash
# With RealSense D435i camera
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py

# Outputs:
# - /visual_slam/tracking/odometry (nav_msgs/Odometry)
# - /visual_slam/tracking/slam_path (nav_msgs/Path)
# - /visual_slam/vis/landmarks_cloud (sensor_msgs/PointCloud2)
```

**Visualize in RViz:**
```bash
rviz2 -d $(ros2 pkg prefix isaac_ros_visual_slam)/share/isaac_ros_visual_slam/rviz/default.rviz
```

**Expected:**
- Real-time camera pose trajectory
- 3D landmark point cloud
- Odometry updates at 100+ Hz

**Custom Integration:**
```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class SlamOdometryTracker(Node):
    def __init__(self):
        super().__init__('slam_tracker')
        self.subscription = self.create_subscription(
            Odometry,
            '/visual_slam/tracking/odometry',
            self.odom_callback,
            10
        )
        self.trajectory = []

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.trajectory.append((pos.x, pos.y, pos.z))

        # Check if robot moved > 1m from start
        if len(self.trajectory) > 1:
            start = self.trajectory[0]
            current = self.trajectory[-1]
            dist = ((current[0]-start[0])**2 +
                    (current[1]-start[1])**2 +
                    (current[2]-start[2])**2)**0.5

            if dist > 1.0:
                self.get_logger().info(f'Robot traveled {dist:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    node = SlamOdometryTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

#### 6. **AprilTag Detection**

**AprilTags:** Fiducial markers for pose estimation, localization, and object tracking.

**Launch AprilTag Detection:**
```bash
ros2 launch isaac_ros_apriltag isaac_ros_apriltag.launch.py
```

**Custom Node: Navigate to AprilTag:**
```python
import rclpy
from rclpy.node import Node
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist

class AprilTagNavigator(Node):
    def __init__(self):
        super().__init__('apriltag_navigator')

        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.tag_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def tag_callback(self, msg):
        if not msg.detections:
            return

        # Get first detected tag
        tag = msg.detections[0]

        # Extract position
        x = tag.pose.pose.pose.position.x
        y = tag.pose.pose.pose.position.y
        z = tag.pose.pose.pose.position.z

        # Simple proportional controller
        cmd = Twist()

        # Move forward if tag is far (z > 0.5m)
        if z > 0.5:
            cmd.linear.x = 0.2
            # Turn to center tag (x should be 0)
            cmd.angular.z = -0.5 * x
        else:
            # Stop when close
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info(f'Reached tag at {z:.2f}m')

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

#### 7. **Integration with Custom Nodes**

**Pattern: Perception → Decision → Action**

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

class PerceptionActionNode(Node):
    def __init__(self):
        super().__init__('perception_action')

        # Subscribe to Isaac ROS processed image
        self.image_sub = self.create_subscription(
            Image,
            '/isaac_ros/image_processed',
            self.image_callback,
            10
        )

        # Publish robot commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Simple color-based detection (detect red objects)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_red = (0, 100, 100)
        upper_red = (10, 255, 255)
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Decision: move toward largest red object
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)

            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])  # Center x
                image_width = cv_image.shape[1]

                # Proportional control: turn toward object
                cmd = Twist()
                cmd.linear.x = 0.2
                cmd.angular.z = 0.5 * (image_width / 2 - cx) / (image_width / 2)

                self.cmd_pub.publish(cmd)
                self.get_logger().info(f'Tracking object at x={cx}')

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionActionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Key Takeaways (7 items)

1. **Isaac ROS provides GPU-accelerated perception** with 10-100x performance improvements
2. **GEMs are reusable building blocks** optimized for NVIDIA hardware (Jetson, RTX)
3. **Docker-based development** simplifies dependency management
4. **cuVSLAM enables real-time VSLAM** at 100+ FPS with loop closure
5. **DOPE provides 6D object pose estimation** for known objects
6. **AprilTags are useful for localization** and object tracking
7. **Integration with custom nodes is straightforward** via standard ROS 2 interfaces

---

### Exercises (4 questions)

#### Conceptual (2)
1. **GPU acceleration**: Explain why GPU acceleration provides such significant speedups (10-100x) for perception tasks like image processing and object detection.

2. **cuVSLAM vs traditional SLAM**: What are 3 advantages of cuVSLAM over CPU-based SLAM algorithms (e.g., ORB-SLAM)?

#### Coding (2)
3. **AprilTag navigation**: Extend the `AprilTagNavigator` example to detect multiple tags (IDs 0-9) and navigate to the tag with ID 5 specifically.

4. **Perception pipeline**: Create a node that subscribes to both cuVSLAM odometry (`/visual_slam/tracking/odometry`) and DOPE object detections (`/dope/pose_array`). When a specific object is detected, use the SLAM pose to record the object's location in a global map.

---

### Further Resources

**Official Documentation:**
- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)
- [Isaac ROS GitHub](https://github.com/NVIDIA-ISAAC-ROS)
- [GEM Architecture](https://nvidia-isaac-ros.github.io/concepts/gems.html)

**Tutorials:**
- Isaac ROS Quickstart
- cuVSLAM Tutorial
- DOPE Object Detection Tutorial

**Next:** [Week 10: Reinforcement Learning Basics](./week10-reinforcement-learning)

---

## Week 10: Reinforcement Learning Basics

### File: `week10-reinforcement-learning.md`
**Location:** `/my-website/docs/module-3/week10-reinforcement-learning.md`

**Frontmatter:**
```yaml
---
sidebar_position: 4
title: Week 10 - Reinforcement Learning Basics
---
```

---

### Learning Objectives (6 items)

- Understand Reinforcement Learning (RL) fundamentals (MDP, policy, value)
- Learn the sim-to-real transfer problem and solutions
- Understand Isaac Gym for GPU-accelerated RL training
- Implement a simple RL environment for a robotic task
- Design reward functions for robot behaviors
- Train a basic policy with stable-baselines3

---

### Content Outline

#### 1. **What is Reinforcement Learning?**

**Definition:** RL is learning **what actions to take** in an environment to **maximize cumulative reward**.

**Key Difference from Supervised Learning:**
- **Supervised**: (input, correct output) pairs
- **Reinforcement**: (state, action, reward, next state) sequences

**Robotics Applications:**
- Manipulation (grasping, assembly)
- Locomotion (walking, running)
- Navigation (obstacle avoidance)
- Task planning (multi-step goals)

**Why RL for Robotics?**
- ✅ Learns from trial and error (no manual programming)
- ✅ Handles high-dimensional state spaces
- ✅ Adapts to new environments
- ❌ Sample inefficient (needs many trials)
- ❌ Sim-to-real gap (what works in sim may fail in reality)

---

#### 2. **RL Fundamentals (MDP Framework)**

**Markov Decision Process (MDP):**
- **State (s)**: Robot's current situation (joint angles, position)
- **Action (a)**: What robot can do (joint torques, velocity commands)
- **Reward (r)**: Scalar feedback (1.0 for success, -0.01 for each timestep)
- **Transition (s' | s, a)**: Next state after taking action
- **Policy (π)**: Strategy mapping states to actions

**Goal:** Find optimal policy π* that maximizes expected cumulative reward:
```
π* = argmax_π E[Σ γ^t * r_t]
```
where γ (gamma) is the discount factor (0 < γ < 1).

**Example: Robot Reaching Task**
```
State: [end_effector_x, end_effector_y, end_effector_z, target_x, target_y, target_z]
Action: [joint_1_torque, joint_2_torque, ..., joint_n_torque]
Reward: -distance(end_effector, target)  # Closer = higher reward
```

---

#### 3. **The Sim-to-Real Problem**

**Reality Gap:** Policies trained in simulation often fail on real robots due to:
1. **Physics inaccuracies**: Friction, contact, deformation
2. **Sensor noise**: Real cameras/LIDAR have noise, sim doesn't
3. **Actuator delays**: Real motors have latency
4. **Unmodeled effects**: Cable drag, wear and tear

**Solutions:**

**1. Domain Randomization (DR)**
- Randomize physics parameters (mass, friction, damping)
- Randomize visual appearance (textures, lighting)
- Randomize sensor noise

**Example:**
```python
# Randomize cube mass during training
mass = random.uniform(0.05, 0.5)  # 50g to 500g
cube.set_mass(mass)
```

**2. System Identification**
- Measure real robot parameters
- Tune simulation to match reality

**3. Sim-to-Sim-to-Real**
- Train in basic sim
- Fine-tune in photorealistic sim (Isaac Sim)
- Deploy to real robot

**4. Residual RL**
- Train base policy in sim
- Fine-tune on real robot with limited data

---

#### 4. **Isaac Gym Overview**

**What is Isaac Gym?**
- GPU-accelerated physics simulator for RL
- Simulates **thousands of parallel environments** on one GPU
- Integrated with PyTorch for end-to-end GPU training

**Why Isaac Gym?**
- **Speed**: 1000x faster than CPU-based RL (MuJoCo, PyBullet)
- **Scale**: Train 4096 robots in parallel
- **Integration**: Direct PyTorch tensors (no CPU↔GPU transfers)

**Architecture:**
```
Isaac Gym (PhysX GPU)
    ↓ (tensors)
PyTorch Neural Network (GPU)
    ↓ (actions)
Isaac Gym (apply actions, step physics)
```

**Comparison: Traditional RL vs Isaac Gym**

| Aspect | CPU-based (MuJoCo) | Isaac Gym |
|--------|-------------------|-----------|
| **Environments/GPU** | 1 | 4096+ |
| **Training Time (humanoid walk)** | 48 hours | 30 minutes |
| **Hardware** | Multi-core CPU | Single GPU |
| **Framework** | OpenAI Gym | PyTorch native |

---

#### 5. **Simple RL Environment: Reaching Task**

**Task:** Train a 2-DOF robot arm to reach a target position.

**Environment Code (Conceptual):**

```python
import numpy as np
import gym
from gym import spaces

class ReachingEnv(gym.Env):
    """2-DOF robot arm reaching environment"""

    def __init__(self):
        super().__init__()

        # State: [joint1_angle, joint2_angle, target_x, target_y]
        self.observation_space = spaces.Box(
            low=np.array([-np.pi, -np.pi, -2, -2]),
            high=np.array([np.pi, np.pi, 2, 2]),
            dtype=np.float32
        )

        # Action: [joint1_torque, joint2_torque]
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(2,),
            dtype=np.float32
        )

        # Robot parameters
        self.link1_length = 1.0
        self.link2_length = 1.0

        self.reset()

    def reset(self):
        # Random initial joint angles
        self.joint_angles = np.random.uniform(-np.pi/2, np.pi/2, 2)

        # Random target position
        self.target = np.random.uniform(-1.5, 1.5, 2)

        return self._get_obs()

    def _get_obs(self):
        return np.concatenate([self.joint_angles, self.target])

    def _forward_kinematics(self):
        """Compute end-effector position from joint angles"""
        q1, q2 = self.joint_angles

        x = self.link1_length * np.cos(q1) + \
            self.link2_length * np.cos(q1 + q2)
        y = self.link1_length * np.sin(q1) + \
            self.link2_length * np.sin(q1 + q2)

        return np.array([x, y])

    def step(self, action):
        # Apply torques (simplified dynamics)
        self.joint_angles += action * 0.1  # Scale action

        # Clip to joint limits
        self.joint_angles = np.clip(self.joint_angles, -np.pi, np.pi)

        # Compute end-effector position
        ee_pos = self._forward_kinematics()

        # Compute reward (negative distance to target)
        distance = np.linalg.norm(ee_pos - self.target)
        reward = -distance

        # Success bonus
        if distance < 0.1:
            reward += 10.0
            done = True
        else:
            done = False

        # Max episode length
        self.timestep += 1
        if self.timestep > 100:
            done = True

        return self._get_obs(), reward, done, {}

    def render(self, mode='human'):
        pass  # Implement visualization if needed

# Usage
env = ReachingEnv()
obs = env.reset()

for _ in range(100):
    action = env.action_space.sample()  # Random action
    obs, reward, done, info = env.step(action)

    if done:
        obs = env.reset()
```

---

#### 6. **Reward Function Design**

**Good Reward Functions:**
- **Dense**: Provide feedback at every step (not just terminal)
- **Shaped**: Guide agent toward goal
- **Normalized**: Keep rewards in similar scale

**Example: Manipulation Task**

```python
def compute_reward(state, action):
    # 1. Distance to object (dense reward)
    ee_pos = state['end_effector_position']
    obj_pos = state['object_position']
    distance_reward = -np.linalg.norm(ee_pos - obj_pos)

    # 2. Grasping bonus
    if state['gripper_closed'] and state['object_grasped']:
        grasp_reward = 5.0
    else:
        grasp_reward = 0.0

    # 3. Success bonus (object at target)
    if np.linalg.norm(obj_pos - state['target_position']) < 0.05:
        success_reward = 20.0
    else:
        success_reward = 0.0

    # 4. Action penalty (encourage smooth motions)
    action_penalty = -0.01 * np.sum(np.abs(action))

    # Total reward
    reward = distance_reward + grasp_reward + success_reward + action_penalty

    return reward
```

**Common Pitfalls:**
- ❌ **Sparse rewards**: Only reward at success (agent never learns)
- ❌ **Reward hacking**: Agent exploits loophole (e.g., shakes object instead of grasping)
- ❌ **Unbalanced scales**: One term dominates others

---

#### 7. **Training with Stable-Baselines3**

**Stable-Baselines3:** Easy-to-use RL library built on PyTorch.

**Install:**
```bash
pip install stable-baselines3[extra]
```

**Train PPO (Proximal Policy Optimization) Agent:**

```python
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

# Create vectorized environment (4 parallel envs)
vec_env = make_vec_env(lambda: ReachingEnv(), n_envs=4)

# Create PPO agent
model = PPO(
    "MlpPolicy",  # Multi-layer perceptron policy
    vec_env,
    verbose=1,
    learning_rate=3e-4,
    n_steps=2048,
    batch_size=64,
    n_epochs=10,
    tensorboard_log="./logs/"
)

# Train for 100k timesteps
model.learn(total_timesteps=100000)

# Save model
model.save("reaching_ppo")

# Load and test
model = PPO.load("reaching_ppo")
env = ReachingEnv()

obs = env.reset()
for _ in range(100):
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, done, info = env.step(action)

    if done:
        obs = env.reset()
```

**Expected Output:**
```
Episode 1: reward=-12.3
Episode 100: reward=-5.1
Episode 500: reward=8.7  # Success!
```

**Monitoring Training:**
```bash
tensorboard --logdir ./logs/
# Open http://localhost:6006
```

---

#### 8. **Sim-to-Real Transfer Tips**

**Best Practices:**

1. **Domain Randomization During Training**
```python
# Randomize physics parameters
mass = random.uniform(0.8, 1.2) * nominal_mass
friction = random.uniform(0.5, 1.5) * nominal_friction
damping = random.uniform(0.9, 1.1) * nominal_damping

# Randomize observations (add noise)
obs_noisy = obs + np.random.normal(0, 0.01, obs.shape)
```

2. **Observation Noise Augmentation**
```python
# Simulate sensor noise during training
camera_noise = np.random.normal(0, 0.02, image.shape)
image_noisy = np.clip(image + camera_noise, 0, 1)
```

3. **Action Delay Simulation**
```python
# Simulate 50ms actuator delay
action_buffer = []
action_buffer.append(action)
if len(action_buffer) > 5:  # 10 Hz control, 50ms = 5 steps @ 100 Hz physics
    delayed_action = action_buffer.pop(0)
else:
    delayed_action = np.zeros_like(action)
```

4. **Conservative Training**
```python
# Limit max joint velocities during training
action_clipped = np.clip(action, -0.5, 0.5)  # Conservative limits
```

---

### Key Takeaways (8 items)

1. **RL learns policies through trial and error** without explicit programming
2. **MDP framework** defines states, actions, rewards, and transitions
3. **Reward function design is critical** - use dense, shaped rewards
4. **Sim-to-real gap is a major challenge** - use domain randomization
5. **Isaac Gym enables massively parallel training** (1000x speedup)
6. **Stable-Baselines3 simplifies RL training** with production-ready algorithms
7. **PPO is a good default algorithm** for continuous control
8. **Successful sim-to-real requires careful environment design** and observation noise

---

### Exercises (5 questions)

#### Conceptual (2)
1. **Sparse vs dense rewards**: Explain why a sparse reward (1.0 for success, 0.0 otherwise) makes learning harder than a dense reward (-distance_to_goal at every step). Provide a concrete example.

2. **Domain randomization**: You trained a grasping policy in Isaac Gym but it fails on the real robot because objects slip out of the gripper. What 3 physics parameters should you randomize during training to improve sim-to-real transfer?

#### Coding (3)
3. **Reward shaping**: Modify the `ReachingEnv` reward function to include:
   - Bonus for low joint velocities (encourage smooth motion)
   - Penalty for exceeding joint limits
   - Bonus for reaching target quickly (inverse of timesteps)

4. **Custom environment**: Create a `BalancingEnv` where a single inverted pendulum (like a segway) must balance upright. Define state space, action space, and reward function.

5. **Training**: Use Stable-Baselines3 to train a PPO agent on your `BalancingEnv` for 50k timesteps. Plot the reward curve and determine if the agent learned to balance.

---

### Further Resources

**RL Foundations:**
- "Reinforcement Learning: An Introduction" (Sutton & Barto) - [Free online](http://incompleteideas.net/book/the-book.html)
- OpenAI Spinning Up in Deep RL - [spinningup.openai.com](https://spinningup.openai.com/)

**Isaac Gym:**
- [Isaac Gym Documentation](https://developer.nvidia.com/isaac-gym)
- Isaac Gym Examples (GitHub)

**Stable-Baselines3:**
- [SB3 Documentation](https://stable-baselines3.readthedocs.io/)
- [SB3 RL Zoo](https://github.com/DLR-RM/rl-baselines3-zoo)

**Next:** Module 4 - Conversational AI (Week 13)

---

## Module 3 Intro Update

### File: `intro.md` (Update)
**Location:** `/my-website/docs/module-3/intro.md`

**Changes Required:**

Replace current content with:

```markdown
# Module 3: The AI-Robot Brain (NVIDIA Isaac)

Welcome to Module 3, where you'll build perception pipelines using NVIDIA Isaac for synthetic data generation, GPU-accelerated perception, and reinforcement learning fundamentals.

**Duration:** 3 weeks (Weeks 8-10)

## What You'll Build

By the end of this module, you will have:
- 🎨 **Synthetic dataset generator** with domain randomization in Isaac Sim
- 👁️ **Real-time perception pipeline** using Isaac ROS (VSLAM, object detection)
- 🤖 **RL-trained policy** for a robotic manipulation task
- 🧠 **GPU-accelerated navigation** with cuVSLAM + Nav2

## What You'll Learn

In this module, you'll explore:

- **Week 8: Isaac Sim Fundamentals** - Photorealistic simulation, synthetic data generation, Python API
- **Week 9: Isaac ROS Integration** - GPU-accelerated perception, cuVSLAM, DOPE object detection
- **Week 10: Reinforcement Learning Basics** - MDP framework, reward design, sim-to-real transfer

## Prerequisites

Before starting this module, you should have:

- ✅ Completed Modules 1-2 (ROS 2 and Simulation)
- ✅ Understanding of simulation and digital twins
- ✅ NVIDIA GPU with 8GB+ VRAM (RTX 3060 or better) OR cloud GPU instance
- ✅ Python 3.10+
- ✅ Ubuntu 22.04 (recommended)

**Cloud Alternative:** AWS G4dn instances or NVIDIA NGC

## Weekly Roadmap

### [Week 8: Isaac Sim Fundamentals](./week8-isaac-sim-fundamentals)

**Learning Objectives:**
- Understand NVIDIA Isaac Sim architecture and capabilities
- Install and configure Isaac Sim (Omniverse platform)
- Generate synthetic training data for vision models
- Simulate cameras, LIDAR, and sensors with realistic physics
- Use Python API for programmatic scene control
- Export datasets for machine learning pipelines

**Topics Covered:**
- Why Isaac Sim? (photorealism, physics, AI integration)
- Installation and setup (Omniverse)
- Synthetic data generation workflow
- Domain randomization for sim-to-real transfer
- Sensor simulation (camera, LIDAR, IMU)
- Python API for automation
- Data export (COCO, YOLO formats)

**Key Deliverable:** Synthetic dataset with 100+ randomized scenes

---

### [Week 9: Isaac ROS Integration](./week9-isaac-ros-integration)

**Learning Objectives:**
- Understand Isaac ROS architecture and GEMs (GPU-Accelerated Extensible Modules)
- Install and configure Isaac ROS packages
- Build GPU-accelerated perception pipelines
- Integrate Isaac ROS with custom ROS 2 nodes
- Perform real-time object detection with NVIDIA models
- Understand VSLAM (Visual Simultaneous Localization and Mapping)

**Topics Covered:**
- Isaac ROS packages overview
- Installation (Docker-based workflow)
- GPU acceleration benefits (10-100x speedup)
- cuVSLAM for real-time localization
- DOPE for 6D object pose estimation
- AprilTag detection for fiducial tracking
- Integration with custom perception pipelines

**Key Deliverable:** Real-time VSLAM + object detection pipeline

---

### [Week 10: Reinforcement Learning Basics](./week10-reinforcement-learning)

**Learning Objectives:**
- Understand Reinforcement Learning (RL) fundamentals (MDP, policy, value)
- Learn the sim-to-real transfer problem and solutions
- Understand Isaac Gym for GPU-accelerated RL training
- Implement a simple RL environment for a robotic task
- Design reward functions for robot behaviors
- Train a basic policy with stable-baselines3

**Topics Covered:**
- RL fundamentals (MDP, policy, reward)
- Sim-to-real gap and domain randomization
- Isaac Gym architecture (massively parallel training)
- Simple RL environment (reaching task)
- Reward function design best practices
- Training with Stable-Baselines3 (PPO)
- Sim-to-real transfer strategies

**Key Deliverable:** Trained RL policy for robot manipulation

---

## Learning Outcomes

By the end of this module, you will be able to:

1. 🎯 Set up Isaac Sim and generate synthetic training datasets with domain randomization
2. 🧠 Implement GPU-accelerated VSLAM with Isaac ROS cuVSLAM
3. 👁️ Build real-time object detection pipelines with DOPE
4. 🤖 Design RL environments and reward functions for robotic tasks
5. 🔄 Train RL policies with PPO using Stable-Baselines3
6. ✅ Understand sim-to-real transfer challenges and solutions

## Module Structure

```
Week 8: Synthetic Data (Isaac Sim)
  ↓
  • Photorealistic simulation
  • Domain randomization
  • Python API automation
  ↓
Week 9: Perception (Isaac ROS)
  ↓
  • GPU-accelerated pipelines
  • cuVSLAM, DOPE, AprilTags
  • ROS 2 integration
  ↓
Week 10: Learning (Reinforcement Learning)
  ↓
  • MDP framework
  • Reward design
  • Policy training
  ↓
Complete AI-Robot Brain
```

## Hardware Requirements

**Minimum Specifications:**
- **GPU**: NVIDIA RTX 3060 (12GB VRAM)
- **CPU**: 8-core processor (AMD Ryzen 7 / Intel i7)
- **RAM**: 32GB
- **Storage**: 50GB free space (SSD recommended)
- **OS**: Ubuntu 22.04 (recommended) or Windows 10/11

**Recommended Specifications:**
- **GPU**: NVIDIA RTX 4070+ (16GB+ VRAM)
- **RAM**: 64GB
- **Storage**: 100GB SSD

**Cloud Alternative:**
- AWS G4dn instances (NVIDIA T4 GPUs)
- NVIDIA NGC cloud deployments
- Google Cloud with NVIDIA GPUs

## Getting Started

Ready to build the AI-Robot Brain? Let's begin with **Week 8: Isaac Sim Fundamentals**!

---

**Next**: [Week 8: Isaac Sim Fundamentals](./week8-isaac-sim-fundamentals)
```

---

## Validation Checkpoint 3

**After completing Phase 3, perform:**

### 1. Build Test
```bash
cd my-website
npm run build
```

**Expected:**
- ✅ Build succeeds without errors
- ✅ No broken links reported
- ✅ Static files generated in `build/`

### 2. Content Validation

**Module 3 Files:**
- ✅ `week8-isaac-sim-fundamentals.md` exists (~900 lines)
- ✅ `week9-isaac-ros-integration.md` exists (~850 lines)
- ✅ `week10-reinforcement-learning.md` exists (~800 lines)
- ✅ `intro.md` updated with weekly roadmap

**Build Artifacts:**
- ✅ `build/docs/module-3/week8-isaac-sim-fundamentals/` exists
- ✅ `build/docs/module-3/week9-isaac-ros-integration/` exists
- ✅ `build/docs/module-3/week10-reinforcement-learning/` exists

### 3. Link Validation

**Internal Links:**
- ✅ Module 3 intro → Week 8, 9, 10
- ✅ Week 8 → Week 9
- ✅ Week 9 → Week 10
- ✅ Week 10 → Module 4

### 4. Quality Check

- ✅ Frontmatter correct (sidebar_position, title)
- ✅ Code examples complete and runnable (or marked as conceptual)
- ✅ Exercises meaningful (conceptual + coding)
- ✅ Key takeaways present (6-8 items)
- ✅ Further resources section
- ✅ Consistent style with previous modules

### 5. Technical Accuracy

- ✅ Isaac Sim API usage correct
- ✅ Isaac ROS package names and topics correct
- ✅ RL concepts accurate (MDP, PPO, reward design)
- ✅ Python code syntax valid
- ✅ Installation instructions current

---

## Success Criteria

**Phase 3 is complete when:**

1. All 3 weekly content files created (~2,550 lines total)
2. Module 3 intro updated with weekly roadmap
3. Build passes without errors
4. All internal links functional
5. Content follows Phase 1 & 2 style
6. Code examples complete and documented
7. Validation checkpoint passes

---

## Notes

**GPU Requirements:**
- Some examples require NVIDIA GPU
- Provide cloud alternatives (AWS, NGC)
- Mark GPU-required sections clearly
- Provide CPU fallbacks where possible

**Isaac Sim Licensing:**
- Free for individual developers
- Requires NVIDIA account
- Commercial use requires license

**Scope Management:**
- Keep RL content high-level (not full research-level implementations)
- Focus on practical robotics applications
- Provide external resources for deep dives

---

**Status:** Ready for implementation
**Estimated Completion:** 3-5 days (depending on content depth)
**Next Phase:** Phase 4 (Final navigation and deployment)
