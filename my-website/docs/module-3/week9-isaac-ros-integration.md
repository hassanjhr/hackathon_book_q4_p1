---
sidebar_position: 3
title: Week 9 - Isaac ROS Integration
---

# Week 9: Isaac ROS Integration

Welcome to Week 9! This week, you'll learn how to use **Isaac ROS** — NVIDIA's collection of **GPU-accelerated ROS 2 packages** for robotics perception. Isaac ROS provides 10-100x performance improvements over traditional CPU-based perception algorithms, enabling real-time performance at high frame rates.

By the end of this week, you'll be running GPU-accelerated Visual SLAM (cuVSLAM), object pose estimation (DOPE), and AprilTag detection on NVIDIA hardware (Jetson or RTX GPUs).

---

## Learning Objectives

By the end of this week, you will be able to:

1. Understand Isaac ROS architecture and GEM (GPU-Accelerated Extensible Modules)
2. Install and configure Isaac ROS packages
3. Build GPU-accelerated perception pipelines
4. Integrate Isaac ROS with custom ROS 2 nodes
5. Perform real-time object detection with NVIDIA models
6. Understand VSLAM (Visual Simultaneous Localization and Mapping)

---

## What is Isaac ROS?

### Definition

**Isaac ROS** is a collection of **GPU-accelerated ROS 2 packages** for robotics perception, navigation, and manipulation. Instead of running perception algorithms on the CPU (which can be slow for real-time tasks), Isaac ROS offloads computation to NVIDIA GPUs using optimized kernels (CUDA, TensorRT).

### Key Features

1. **GPU Acceleration**: 10-100x faster than CPU-based implementations
2. **NVIDIA Hardware-Specific**: Optimized for Jetson (embedded) and RTX GPUs (desktop/server)
3. **Production-Ready**: Battle-tested for real-world deployment
4. **Modular**: Use individual packages or full perception pipelines
5. **ROS 2 Native**: Standard ROS 2 interfaces (topics, services, actions)

---

### GEMs: GPU-Accelerated Extensible Modules

**GEMs** are reusable building blocks for perception pipelines:
- Pre-built, optimized kernels (CUDA, TensorRT)
- Standard ROS 2 interfaces (subscribe to images, publish detections)
- Composable (chain multiple GEMs together)

**Architecture:**
```
ROS 2 Node (Python/C++)
    ↓ (sensor_msgs/Image)
Isaac ROS GEM (GPU-accelerated)
    ↓ (TensorRT inference, CUDA processing)
NVIDIA Hardware (Jetson Orin, RTX 4090)
    ↓ (outputs)
ROS 2 Topic (detections, poses, point clouds)
```

---

### When to Use Isaac ROS?

**Use Isaac ROS when:**
- ✅ You have NVIDIA hardware (Jetson or RTX GPU)
- ✅ You need real-time performance (> 30 FPS)
- ✅ You're deploying perception to production
- ✅ You want to minimize latency (< 50ms)

**Use traditional ROS 2 packages when:**
- ❌ No NVIDIA GPU available
- ❌ Prototyping/research (CPU is "good enough")
- ❌ Battery-constrained systems (GPU power consumption)

---

## Isaac ROS Packages Overview

### Key Packages

| Package | Purpose | Input → Output | Use Case |
|---------|---------|----------------|----------|
| **isaac_ros_visual_slam** | cuVSLAM | Stereo/mono camera → Odometry + map | Localization & mapping |
| **isaac_ros_dnn_inference** | TensorRT inference | Image → Tensor | Object detection, segmentation |
| **isaac_ros_image_proc** | Image processing | Raw image → Rectified image | Debayering, rectification |
| **isaac_ros_apriltag** | AprilTag detection | Image → Tag poses | Fiducial markers |
| **isaac_ros_depth_image_proc** | Depth processing | Depth image → Point cloud | 3D reconstruction |
| **isaac_ros_nvblox** | 3D reconstruction | Depth stream → Occupancy map | Real-time mapping |
| **isaac_ros_pose_estimation** | DOPE (6D pose) | Image → Object poses | Grasping, manipulation |

---

### Performance Comparison: ROS 2 vs Isaac ROS

| Task | ROS 2 (CPU) | Isaac ROS (GPU) | Speedup |
|------|-------------|-----------------|---------|
| **Image rectification** | 30 FPS | 200+ FPS | 6-7x |
| **Object detection (YOLOv8)** | 5 FPS (i7 CPU) | 60+ FPS (RTX 3060) | 12x |
| **Visual SLAM** | 10 FPS (ORB-SLAM3) | 100+ FPS (cuVSLAM) | 10x |
| **Point cloud processing** | 5 FPS | 50+ FPS | 10x |
| **AprilTag detection** | 20 FPS | 200+ FPS | 10x |

*Note: Exact speedups depend on hardware and image resolution.*

---

## Installation

### Prerequisites

**Hardware:**
- **NVIDIA GPU**: Jetson Orin, AGX Xavier, RTX 3060+, or A-series
- **Driver**: NVIDIA Driver 525+ (for RTX), JetPack 5.0+ (for Jetson)

**Software:**
- **OS**: Ubuntu 22.04 (recommended)
- **ROS 2**: Humble Hawksbill
- **Docker**: Latest version
- **CUDA**: 12.0+ (for RTX), included with JetPack (for Jetson)
- **TensorRT**: 8.6+ (bundled with Isaac ROS Docker image)

---

### Installation Steps (Docker-Based)

Isaac ROS uses **Docker** to simplify dependency management (CUDA, TensorRT, ROS 2).

#### 1. Install Docker

```bash
# Install Docker Engine
curl https://get.docker.com | sh

# Start Docker service
sudo systemctl --now enable docker

# Add user to docker group (avoid sudo for docker commands)
sudo usermod -aG docker $USER
newgrp docker  # Apply group change without logout
```

#### 2. Clone Isaac ROS Common

```bash
# Create workspace
mkdir -p ~/workspaces/isaac_ros-dev/src
cd ~/workspaces/isaac_ros-dev/src

# Clone Isaac ROS common repository
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
```

#### 3. Build and Enter Docker Container

```bash
# Navigate to isaac_ros_common
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common

# Run development container (downloads ~10GB on first run)
./scripts/run_dev.sh

# This will:
# - Build a Docker image with ROS 2 + CUDA + TensorRT
# - Mount your workspace inside the container
# - Start a shell inside the container
```

#### 4. Install Isaac ROS Packages (Inside Container)

```bash
# Inside Docker container
cd /workspaces/isaac_ros-dev

# Clone desired Isaac ROS packages (example: Visual SLAM)
cd src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git

# Build packages
cd /workspaces/isaac_ros-dev
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

#### 5. Verify Installation

```bash
# List installed Isaac ROS packages
ros2 pkg list | grep isaac_ros

# Expected output:
# isaac_ros_visual_slam
# isaac_ros_nvblox
# isaac_ros_apriltag
# ...
```

---

## Example 1: Object Detection with DOPE

**DOPE (Deep Object Pose Estimation)** detects **3D pose** (position + orientation) of known objects from RGB images.

### Use Case
- Grasping (robot needs to know where object is in 3D)
- Bin picking (detect pose of parts in a bin)
- AR/VR (overlay virtual objects on real ones)

---

### Install DOPE Package

```bash
# Inside Docker container
cd ~/workspaces/isaac_ros-dev/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation.git

cd /workspaces/isaac_ros-dev
colcon build --packages-up-to isaac_ros_dope
source install/setup.bash
```

---

### Download Pre-Trained Model

DOPE requires a trained model for each object type (e.g., "ketchup bottle", "soup can").

```bash
# Download NVIDIA pre-trained models
mkdir -p ~/workspaces/isaac_ros-dev/models
cd ~/workspaces/isaac_ros-dev/models

# Download Ketchup model (example)
wget https://github.com/NVlabs/Deep_Object_Pose/raw/master/weights/ketchup.pth

# Convert to ONNX (required for TensorRT)
# (DOPE package includes conversion scripts)
```

---

### Launch DOPE

```bash
# Terminal 1: Start USB camera (or use RealSense)
ros2 run usb_cam usb_cam_node_exe --ros-args -p image_width:=640 -p image_height:=480

# Terminal 2: Launch DOPE
ros2 launch isaac_ros_dope isaac_ros_dope_tensor_rt.launch.py \
    model_file_path:=/workspaces/isaac_ros-dev/models/ketchup.onnx \
    engine_file_path:=/workspaces/isaac_ros-dev/models/ketchup.plan

# Terminal 3: Visualize in RViz
rviz2
# Add: PoseArray topic -> /dope/pose_array
```

---

### Custom Python Subscriber: Listen to DOPE Poses

```python
"""
DOPE Listener: Subscribe to detected object poses.
"""
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

        self.get_logger().info('DOPE Listener started. Waiting for detections...')

    def pose_callback(self, msg):
        if not msg.poses:
            self.get_logger().info('No objects detected')
            return

        for i, pose in enumerate(msg.poses):
            pos = pose.position
            ori = pose.orientation

            self.get_logger().info(
                f'Object {i}: Position=({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}), '
                f'Orientation=({ori.x:.3f}, {ori.y:.3f}, {ori.z:.3f}, {ori.w:.3f})'
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

**Run:**
```bash
python3 dope_listener.py
```

**Expected Output:**
```
[INFO] [dope_listener]: DOPE Listener started. Waiting for detections...
[INFO] [dope_listener]: Object 0: Position=(0.234, -0.156, 0.823), Orientation=(0.001, -0.002, 0.707, 0.707)
```

---

## Example 2: Visual SLAM with cuVSLAM

**cuVSLAM** (CUDA Visual SLAM) is a **GPU-accelerated Visual SLAM** algorithm for real-time localization and mapping.

### Features
- **Stereo or monocular** camera support
- **IMU fusion** for improved accuracy
- **Loop closure detection** for drift correction
- **100+ FPS** performance on RTX GPUs

---

### Install cuVSLAM

```bash
# Inside Docker container
cd ~/workspaces/isaac_ros-dev/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git

cd /workspaces/isaac_ros-dev
colcon build --packages-up-to isaac_ros_visual_slam
source install/setup.bash
```

---

### Launch cuVSLAM with RealSense Camera

```bash
# With Intel RealSense D435i (stereo + IMU)
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py

# Outputs:
# - /visual_slam/tracking/odometry (nav_msgs/Odometry)
# - /visual_slam/tracking/slam_path (nav_msgs/Path)
# - /visual_slam/vis/landmarks_cloud (sensor_msgs/PointCloud2)
```

---

### Visualize in RViz

```bash
# Open RViz with default config
rviz2 -d $(ros2 pkg prefix isaac_ros_visual_slam)/share/isaac_ros_visual_slam/rviz/default.rviz
```

**Expected Visualization:**
- **Camera pose trajectory** (green line)
- **3D landmark point cloud** (colored points)
- **Odometry updates** at 100+ Hz

---

### Custom Integration: Track Robot Trajectory

```python
"""
SLAM Odometry Tracker: Subscribe to cuVSLAM odometry and track traveled distance.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class SlamOdometryTracker(Node):
    def __init__(self):
        super().__init__('slam_tracker')

        self.subscription = self.create_subscription(
            Odometry,
            '/visual_slam/tracking/odometry',
            self.odom_callback,
            10
        )

        self.trajectory = []  # List of (x, y, z) tuples
        self.get_logger().info('SLAM Tracker started')

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.trajectory.append((pos.x, pos.y, pos.z))

        # Check if robot moved > 1m from start
        if len(self.trajectory) > 1:
            start = self.trajectory[0]
            current = self.trajectory[-1]

            dist = math.sqrt(
                (current[0] - start[0])**2 +
                (current[1] - start[1])**2 +
                (current[2] - start[2])**2
            )

            if dist > 1.0 and len(self.trajectory) % 100 == 0:
                self.get_logger().info(f'Robot traveled {dist:.2f}m from start')

def main(args=None):
    rclpy.init(args=args)
    node = SlamOdometryTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Output:**
```
[INFO] [slam_tracker]: SLAM Tracker started
[INFO] [slam_tracker]: Robot traveled 1.23m from start
[INFO] [slam_tracker]: Robot traveled 2.45m from start
```

---

## Example 3: AprilTag Detection

**AprilTags** are fiducial markers (like QR codes) used for:
- Robot localization (known tag positions)
- Object tracking (tags attached to objects)
- Camera calibration

Isaac ROS provides **GPU-accelerated AprilTag detection** at 200+ FPS.

---

### Install AprilTag Package

```bash
# Inside Docker container
cd ~/workspaces/isaac_ros-dev/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git

cd /workspaces/isaac_ros-dev
colcon build --packages-up-to isaac_ros_apriltag
source install/setup.bash
```

---

### Launch AprilTag Detection

```bash
# Terminal 1: Start camera
ros2 run usb_cam usb_cam_node_exe

# Terminal 2: Launch AprilTag detection
ros2 launch isaac_ros_apriltag isaac_ros_apriltag.launch.py

# Terminal 3: Visualize
rviz2
# Add: TF frames, Image topic -> /tag_detections_image
```

---

### Custom Node: Navigate to AprilTag

This example shows a simple proportional controller that moves a robot toward a detected AprilTag.

```python
"""
AprilTag Navigator: Move robot toward detected AprilTag.
"""
import rclpy
from rclpy.node import Node
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist

class AprilTagNavigator(Node):
    def __init__(self):
        super().__init__('apriltag_navigator')

        # Subscribe to AprilTag detections
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.tag_callback,
            10
        )

        # Publish velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('AprilTag Navigator started. Looking for tags...')

    def tag_callback(self, msg):
        if not msg.detections:
            # No tags detected, stop
            self.stop_robot()
            return

        # Get first detected tag
        tag = msg.detections[0]

        # Extract 3D position (camera frame)
        x = tag.pose.pose.pose.position.x  # Horizontal offset
        y = tag.pose.pose.pose.position.y  # Vertical offset (not used for 2D navigation)
        z = tag.pose.pose.pose.position.z  # Distance to tag

        # Simple proportional controller
        cmd = Twist()

        # Move forward if tag is far (z > 0.5m)
        if z > 0.5:
            cmd.linear.x = 0.2  # 0.2 m/s forward

            # Turn to center tag (x should be 0)
            # Proportional gain: -0.5
            cmd.angular.z = -0.5 * x

            self.get_logger().info(f'Tag ID {tag.id}: distance={z:.2f}m, offset={x:.2f}m')
        else:
            # Stop when close enough
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info(f'Reached tag {tag.id} at {z:.2f}m!')

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        """Send zero velocity command"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
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

**Behavior:**
- Detects AprilTag in camera view
- Turns to center the tag (minimize x offset)
- Moves forward until distance < 0.5m
- Stops when close enough

---

## Integration with Custom Nodes

Isaac ROS packages output **standard ROS 2 messages**, so you can integrate them with custom decision-making and control logic.

### Common Pattern: Perception → Decision → Action

```python
"""
Perception-Action Integration: Use Isaac ROS image processing for color-based object tracking.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

class PerceptionActionNode(Node):
    def __init__(self):
        super().__init__('perception_action')

        # Subscribe to Isaac ROS processed image (e.g., rectified, denoised)
        self.image_sub = self.create_subscription(
            Image,
            '/isaac_ros/image_rect',  # Rectified image from Isaac ROS
            self.image_callback,
            10
        )

        # Publish robot commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bridge = CvBridge()
        self.get_logger().info('Perception-Action Node started')

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Simple color-based detection (detect red objects)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Red color range (HSV)
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
                # Centroid of red object
                cx = int(M["m10"] / M["m00"])
                image_width = cv_image.shape[1]

                # Proportional control: turn toward object
                cmd = Twist()
                cmd.linear.x = 0.2  # Move forward

                # Error: how far object is from image center
                error = (image_width / 2) - cx
                cmd.angular.z = 0.005 * error  # Proportional gain

                self.cmd_pub.publish(cmd)
                self.get_logger().info(f'Tracking red object at x={cx}')
        else:
            # No red object detected, stop
            self.stop_robot()

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionActionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Pipeline:**
1. **Perception**: Isaac ROS rectifies camera image (GPU-accelerated)
2. **Decision**: Custom Python node detects red objects (OpenCV)
3. **Action**: Publish velocity commands to move toward object

---

## Key Takeaways

1. **Isaac ROS provides GPU-accelerated perception** with 10-100x performance improvements over CPU-based algorithms

2. **GEMs are reusable building blocks** — optimized CUDA/TensorRT kernels exposed as ROS 2 nodes

3. **Docker-based development** simplifies dependency management (CUDA, TensorRT, ROS 2 all pre-configured)

4. **cuVSLAM enables real-time Visual SLAM** at 100+ FPS with stereo cameras and IMU fusion

5. **DOPE provides 6D object pose estimation** for known objects (grasping, manipulation)

6. **AprilTags are useful for localization** and object tracking with GPU-accelerated detection

7. **Integration with custom nodes is straightforward** — Isaac ROS uses standard ROS 2 interfaces (topics, services)

---

## Exercises

### Conceptual (2 questions)

**1. GPU Acceleration Benefits**

Explain why GPU acceleration provides such significant speedups (10-100x) for perception tasks like image processing and object detection. What makes GPUs better than CPUs for these tasks?

<details>
<summary>Answer</summary>

**Parallel Processing:**
- GPUs have thousands of cores (e.g., RTX 4090: 16,384 CUDA cores) vs CPUs with 8-32 cores
- Image processing is **embarrassingly parallel** (each pixel can be processed independently)
- Example: Image rectification with 1920×1080 pixels = 2 million operations → GPU runs 2 million threads in parallel

**Memory Bandwidth:**
- GPUs have higher bandwidth (RTX 4090: 1 TB/s vs CPU: 50 GB/s)
- Perception tasks are memory-intensive (reading/writing large images)

**Specialized Hardware:**
- Tensor Cores for matrix operations (neural network inference)
- Texture units for image filtering
- Hardware video encoders/decoders
</details>

---

**2. cuVSLAM vs Traditional SLAM**

What are **3 advantages** of cuVSLAM over CPU-based SLAM algorithms (e.g., ORB-SLAM3)?

<details>
<summary>Answer</summary>

1. **Speed**: 100+ FPS vs 10-20 FPS (10x faster) — enables real-time performance even on high-resolution cameras

2. **Latency**: < 10ms vs 50-100ms — critical for closed-loop control (robot reacts faster to environment changes)

3. **Scalability**: Can process multiple camera streams simultaneously on one GPU (e.g., 4 stereo cameras at 60 FPS each)

*Bonus:* **Integration**: Direct tensor output to neural networks (no CPU↔GPU transfers), enabling end-to-end learning pipelines.
</details>

---

### Coding (2 exercises)

**3. AprilTag Multi-Target Navigation**

Extend the `AprilTagNavigator` example to:
1. Detect multiple tags (IDs 0-9)
2. Navigate to the tag with **ID 5** specifically (ignore other tags)
3. Print a warning if ID 5 is not detected

<details>
<summary>Hint</summary>

Iterate through `msg.detections`, check `tag.id == 5`, and only move toward that tag. If no ID 5 found, call `stop_robot()` and log a warning.
</details>

---

**4. Perception Pipeline: SLAM + Object Detection**

Create a ROS 2 node that subscribes to **both**:
- cuVSLAM odometry (`/visual_slam/tracking/odometry`)
- DOPE object detections (`/dope/pose_array`)

**Task:**
When a specific object is detected (e.g., "ketchup bottle"), use the SLAM pose to record the object's **global position** (transform object pose from camera frame to world frame).

**Expected Output:**
```
[INFO] [object_mapper]: Detected ketchup at camera position (0.23, -0.15, 0.82)
[INFO] [object_mapper]: Robot pose: (1.50, 2.30, 0.00)
[INFO] [object_mapper]: Object global position: (1.73, 2.15, 0.82)
```

<details>
<summary>Hint</summary>

- Store latest SLAM pose from `/visual_slam/tracking/odometry`
- When DOPE detects object, transform object pose (camera frame) to world frame using SLAM pose
- Use `tf2_ros` for coordinate transformations, or manually apply translation/rotation
</details>

---

## Further Resources

**Official Documentation:**
- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/) - Complete reference
- [Isaac ROS GitHub](https://github.com/NVIDIA-ISAAC-ROS) - All packages and examples
- [GEM Architecture](https://nvidia-isaac-ros.github.io/concepts/gems.html) - How GEMs work

**Video Tutorials:**
- [Isaac ROS Quickstart](https://www.youtube.com/watch?v=xyz) - Official walkthrough
- [cuVSLAM Tutorial](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html)
- [DOPE Object Detection](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_pose_estimation/isaac_ros_dope/index.html)

**Research Papers:**
- *DOPE: Deep Object Pose Estimation* (2018) - Original DOPE paper
- *cuVSLAM: GPU-Accelerated Visual SLAM* (NVIDIA whitepaper)

**Community:**
- [NVIDIA Isaac Forum](https://forums.developer.nvidia.com/c/isaac/) - Ask questions
- [Isaac ROS Issues](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/issues) - Report bugs

---

**Next:** [Week 10: Reinforcement Learning Basics](./week10-reinforcement-learning) - Learn RL fundamentals, Isaac Gym, and train robot policies with Stable-Baselines3.
