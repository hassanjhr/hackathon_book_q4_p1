---
sidebar_position: 2
title: Week 8 - Isaac Sim Fundamentals
---

# Week 8: NVIDIA Isaac Sim Fundamentals

Welcome to Week 8! This week, you'll learn how to use **NVIDIA Isaac Sim**, a photorealistic robot simulator built on NVIDIA Omniverse. Isaac Sim enables GPU-accelerated physics, realistic sensor simulation, and massive-scale synthetic data generation for training AI models.

By the end of this week, you'll be able to generate thousands of labeled training images automatically, simulate realistic sensors (cameras, LIDAR, IMU), and build pipelines for sim-to-real transfer.

---

## Learning Objectives

By the end of this week, you will be able to:

1. Understand NVIDIA Isaac Sim architecture and capabilities
2. Install and configure Isaac Sim (Omniverse platform)
3. Generate synthetic training data for vision models
4. Simulate cameras, LIDAR, and sensors with realistic physics
5. Use Python API for programmatic scene control
6. Export datasets for machine learning pipelines

---

## Why NVIDIA Isaac Sim?

### The Challenge: Real-World Data is Expensive

Training modern AI models (object detection, segmentation, pose estimation) requires **thousands or millions** of labeled images. Collecting and labeling real-world data is:

- **Expensive**: $0.10-$1.00 per image (bounding boxes, segmentation masks)
- **Time-consuming**: Months to collect diverse scenarios
- **Incomplete**: Hard to capture rare edge cases (unusual lighting, occlusions)
- **Unsafe**: Testing dangerous scenarios (collisions, failures) on real robots

**Solution:** Generate **synthetic data** in simulation with perfect labels automatically.

---

### Isaac Sim vs Gazebo: When to Use Each?

| Feature | Gazebo | Isaac Sim |
|---------|--------|-----------|
| **Graphics** | Good (OpenGL) | Photorealistic (RTX ray tracing) |
| **Physics** | ODE/Bullet (CPU) | NVIDIA PhysX (GPU) |
| **Sensors** | Basic simulation | Realistic (lens distortion, noise) |
| **AI/ML Integration** | Limited | Native (TensorRT, PyTorch) |
| **Performance** | CPU-bound | GPU-accelerated (10-100x faster) |
| **Synthetic Data** | Manual scripting | Built-in (Replicator) |
| **Use Case** | Algorithm prototyping, ROS 2 testing | AI training, photorealism, digital twins |
| **Hardware** | Any CPU | NVIDIA GPU (RTX 3060+, Jetson) |

**When to use each:**
- **Gazebo**: Algorithm development, ROS 2 testing, CPU-only systems, educational purposes
- **Isaac Sim**: Training vision models, photorealistic visualization, GPU-accelerated physics, sim-to-real deployment

---

## Isaac Sim Architecture

Isaac Sim is built on **NVIDIA Omniverse Kit**, a platform for 3D workflows using the **USD (Universal Scene Description)** format.

### Key Components

1. **Omniverse Kit**: Foundation layer
   - USD scene graph (Pixar's open standard)
   - Real-time rendering engine
   - Plugin architecture

2. **Isaac Sim**: Robotics-specific features
   - Robot importers (URDF, USD)
   - ROS 2 bridge
   - Sensor simulation
   - Python API for automation

3. **PhysX**: GPU-accelerated physics engine
   - Rigid body dynamics
   - Articulated bodies (robots)
   - Soft bodies, cloth, fluids

4. **RTX Renderer**: Photorealistic ray tracing
   - Global illumination
   - Reflections, refractions
   - Physically-based materials

5. **Replicator**: Synthetic data generation
   - Domain randomization
   - Data writers (COCO, KITTI, etc.)
   - Scalable rendering

### Architecture Diagram

```
┌─────────────────────────────────────────┐
│        Isaac Sim Application            │
├─────────────────────────────────────────┤
│  Python API  │  GUI  │  ROS 2 Bridge    │
├──────────────┴───────┴──────────────────┤
│         PhysX (Physics Engine)           │
│         RTX (Ray Tracing Renderer)       │
│         Replicator (Synthetic Data)      │
├─────────────────────────────────────────┤
│    Omniverse Kit (USD Runtime)          │
└─────────────────────────────────────────┘
```

---

## Installation and Setup

### Prerequisites

**Hardware:**
- **GPU**: NVIDIA RTX 3060+ (12GB+ VRAM recommended)
  - Minimum: RTX 2080, Quadro RTX 5000
  - Ideal: RTX 4090, A6000 (for large scenes)
- **CPU**: Intel i7/i9 or AMD Ryzen 7/9
- **RAM**: 32GB+ recommended
- **Storage**: 50GB+ free space (SSD recommended)

**Software:**
- **OS**: Ubuntu 20.04/22.04 (recommended) or Windows 10/11
- **NVIDIA Driver**: 525+ (latest recommended)
- **Python**: 3.10 (bundled with Isaac Sim)

---

### Installation Steps

#### 1. Install Omniverse Launcher

**Linux:**
```bash
# Download Omniverse Launcher
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Make executable
chmod +x omniverse-launcher-linux.AppImage

# Run launcher
./omniverse-launcher-linux.AppImage
```

**Windows:**
- Download from: https://www.nvidia.com/en-us/omniverse/download/
- Run installer: `omniverse-launcher-win.exe`

---

#### 2. Install Isaac Sim via Launcher

1. **Open Omniverse Launcher**
2. **Install Nucleus** (local asset server)
   - Go to "Nucleus" tab
   - Click "Install" for local Nucleus server
   - Wait for installation (~5 minutes)

3. **Install Isaac Sim**
   - Go to "Exchange" tab
   - Search for "Isaac Sim"
   - Click "Install" (version 2023.1.1 or later)
   - Download size: ~10GB, installation time: ~15 minutes

4. **Launch Isaac Sim**
   - Go to "Library" tab
   - Click "Launch" next to Isaac Sim

---

### Verify Installation (Python Standalone)

Isaac Sim includes a Python interpreter (`python.sh`) with all dependencies pre-installed.

**Find Python path:**
```bash
# Linux
~/.local/share/ov/pkg/isaac_sim-*/python.sh --version

# Windows (PowerShell)
C:\Users\$env:USERNAME\AppData\Local\ov\pkg\isaac_sim-*\python.bat --version
```

**Set alias for convenience (Linux):**
```bash
# Add to ~/.bashrc
alias isaac_python="~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh"

# Reload
source ~/.bashrc
```

---

### First Script: Falling Cube

Create `falling_cube.py`:

```python
"""
Isaac Sim Example: Falling Cube
Creates a red cube at height 1m, simulates gravity.
"""
from isaacsim import SimulationApp

# Initialize simulation (headless=False shows GUI)
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
import numpy as np

# Create world
world = World()
world.scene.add_default_ground_plane()

# Add a dynamic cube (affected by gravity)
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="my_cube",
        position=np.array([0.0, 0.0, 1.0]),  # 1m above ground
        size=0.5,  # 0.5m x 0.5m x 0.5m
        color=np.array([1.0, 0.0, 0.0])  # Red (RGB)
    )
)

# Reset simulation
world.reset()

# Run simulation for 100 steps (≈5 seconds at 20 FPS)
for i in range(100):
    world.step(render=True)

    # Get cube position
    position, orientation = cube.get_world_pose()
    print(f"Step {i}: Cube height = {position[2]:.3f}m")

simulation_app.close()
```

**Run:**
```bash
isaac_python falling_cube.py
```

**Expected Output:**
- Isaac Sim GUI opens
- Red cube falls and bounces on ground plane
- Terminal prints cube height each step

---

## Synthetic Data Generation

### Why Synthetic Data?

**Advantages:**
1. **Cost**: Free vs $0.10-$1.00 per labeled image
2. **Scale**: Generate millions of images automatically
3. **Diversity**: Randomize lighting, textures, object poses
4. **Perfect labels**: Ground truth for segmentation, depth, 6D pose
5. **Rare scenarios**: Simulate edge cases (occlusions, extreme lighting)

**Use Cases:**
- Object detection (YOLO, Faster R-CNN)
- Semantic segmentation (U-Net, DeepLab)
- Depth estimation (monocular, stereo)
- 6D pose estimation (DOPE, PoseCNN)
- Instance segmentation (Mask R-CNN)

---

### Example: Camera Setup and RGB/Depth Capture

```python
"""
Isaac Sim Example: Camera Capture
Captures RGB and depth images from a camera.
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.sensor import Camera
import numpy as np
import matplotlib.pyplot as plt

# Create world
world = World()
world.scene.add_default_ground_plane()

# Load a robot (Franka Panda arm)
add_reference_to_stage(
    usd_path="/Isaac/Robots/Franka/franka_alt_fingers.usd",
    prim_path="/World/Franka"
)

# Add camera
camera = Camera(
    prim_path="/World/Camera",
    position=np.array([2.0, 2.0, 1.5]),  # 2m away, 1.5m high
    frequency=30,  # 30 Hz
    resolution=(1280, 720)  # HD resolution
)

# Point camera at robot
camera.set_local_pose(
    translation=np.array([2.0, 2.0, 1.5]),
    orientation=np.array([0.92, -0.38, 0.0, 0.0])  # Quaternion
)

# Initialize
world.reset()

# Capture 10 frames
for i in range(10):
    world.step(render=True)

    # Get camera data
    rgb_data = camera.get_rgba()  # Shape: (720, 1280, 4)
    depth_data = camera.get_depth()  # Shape: (720, 1280), meters

    print(f"Frame {i}:")
    print(f"  RGB shape: {rgb_data.shape}, dtype: {rgb_data.dtype}")
    print(f"  Depth range: {depth_data.min():.2f}m - {depth_data.max():.2f}m")

    # Save first frame
    if i == 0:
        plt.imsave("rgb_frame0.png", rgb_data[:, :, :3])  # Drop alpha
        plt.imsave("depth_frame0.png", depth_data, cmap='plasma')
        print("  Saved: rgb_frame0.png, depth_frame0.png")

simulation_app.close()
```

**Output:**
- `rgb_frame0.png`: Photorealistic RGB image of Franka robot
- `depth_frame0.png`: Depth map (color-coded by distance)

---

## Domain Randomization

### What is Domain Randomization?

**Problem:** Models trained on synthetic data often fail on real-world data due to the **sim-to-real gap** (different lighting, textures, camera characteristics).

**Solution:** Randomize scene parameters during training to force the model to learn robust features.

### Randomization Strategies

1. **Lighting**
   - Intensity: 500-5000 lumens
   - Color temperature: 3000K (warm) to 6500K (cool)
   - Position: Random placement

2. **Textures and Materials**
   - Object colors: Random RGB
   - Materials: Metallic, plastic, wood, concrete
   - Reflectivity, roughness

3. **Object Poses**
   - Position: Random within bounds
   - Orientation: Random rotation
   - Scale: ±20% variation

4. **Camera Parameters**
   - Position: Random viewpoint
   - Focal length: 20-50mm
   - Noise: Gaussian, salt-and-pepper

5. **Physics (for dynamics)**
   - Friction: 0.2-0.8
   - Mass: ±30% variation
   - Damping coefficients

---

### Code Example: Randomized Scene with Replicator

```python
"""
Isaac Sim Example: Domain Randomization
Generates 100 randomized scenes with cubes.
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni.replicator.core as rep
from omni.isaac.core import World

# Create world
world = World()
world.scene.add_default_ground_plane()
world.reset()

# Define randomization using Replicator
with rep.new_layer():
    # Create 10 cubes with random properties
    cubes = rep.create.cube(
        semantics=[("class", "cube")],  # Class label for ML
        count=10,
        position=rep.distribution.uniform((-2, -2, 0.5), (2, 2, 2)),  # Random XYZ
        scale=rep.distribution.uniform(0.1, 0.5),  # Size: 0.1m - 0.5m
        color=rep.distribution.uniform((0, 0, 0), (1, 1, 1))  # Random RGB
    )

    # Create 3 randomized lights
    lights = rep.create.light(
        light_type="Sphere",
        intensity=rep.distribution.uniform(1000, 5000),  # Lumens
        position=rep.distribution.uniform((-5, -5, 3), (5, 5, 8)),
        color=rep.distribution.uniform((0.8, 0.8, 0.8), (1.0, 1.0, 1.0)),  # Cool-warm
        count=3
    )

    # Create camera with random position
    camera = rep.create.camera(
        position=rep.distribution.uniform((2, 2, 1), (4, 4, 3)),
        look_at=(0, 0, 0)  # Always point at origin
    )

    # Render settings
    render_product = rep.create.render_product(camera, (512, 512))

    # Writer: Save images + bounding boxes
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir="./synthetic_data",
        rgb=True,
        bounding_box_2d_tight=True,  # 2D bbox annotations
        semantic_segmentation=True   # Pixel-wise labels
    )
    writer.attach([render_product])

# Randomize and render 100 frames
with rep.trigger.on_frame(num_frames=100):
    # Re-scatter cubes on ground plane each frame
    rep.randomizer.scatter_2d(
        cubes,
        surface_prims="/World/defaultGroundPlane/geom",
        check_for_collisions=True
    )

# Run orchestrator (generates all frames)
rep.orchestrator.run()

print("Generated 100 randomized scenes in ./synthetic_data/")
simulation_app.close()
```

**Output:**
```
synthetic_data/
├── rgb_0000.png
├── rgb_0001.png
├── ...
├── rgb_0099.png
├── bounding_box_2d_tight_0000.npy
├── ...
├── semantic_segmentation_0000.png
└── ...
```

Each frame has:
- **RGB image**: Randomized cube positions, colors, lighting
- **Bounding boxes**: Tight 2D boxes around each cube
- **Semantic segmentation**: Per-pixel class labels

---

## Sensor Simulation

Isaac Sim supports **realistic sensor simulation** with accurate noise models, lens distortion, and physical properties.

### Supported Sensors

| Sensor | Output | Use Case |
|--------|--------|----------|
| **RGB Camera** | `sensor_msgs/Image` | Object detection, tracking |
| **Depth Camera** | `sensor_msgs/Image` (float32) | 3D reconstruction, SLAM |
| **Semantic Segmentation** | `sensor_msgs/Image` (uint8 labels) | Training segmentation models |
| **Instance Segmentation** | `sensor_msgs/Image` (instance IDs) | Multi-object tracking |
| **LIDAR** | `sensor_msgs/PointCloud2` | Obstacle detection, mapping |
| **IMU** | `sensor_msgs/Imu` | State estimation, localization |
| **Contact Sensor** | `geometry_msgs/WrenchStamped` | Grasping, tactile feedback |

---

### Example: LIDAR Simulation

```python
"""
Isaac Sim Example: LIDAR Simulation
Simulates a rotating 360° LIDAR with 32 beams.
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.range_sensor import _range_sensor
from omni.isaac.core.objects import DynamicCuboid
import numpy as np

# Create world
world = World()
world.scene.add_default_ground_plane()

# Add obstacles (cubes at random positions)
for i in range(5):
    x = np.random.uniform(-3, 3)
    y = np.random.uniform(-3, 3)
    DynamicCuboid(
        prim_path=f"/World/Obstacle_{i}",
        position=np.array([x, y, 0.5]),
        size=0.5,
        color=np.array([0.2, 0.2, 0.8])  # Blue
    )

# Create LIDAR sensor
lidar_config = _range_sensor.acquire_lidar_sensor_interface()

# Configure rotating LIDAR
lidar_path = "/World/Lidar"
result, lidar = omni.kit.commands.execute(
    "RangeSensorCreateLidar",
    path=lidar_path,
    parent=None,
    min_range=0.4,
    max_range=100.0,
    draw_points=True,  # Visualize points in viewport
    draw_lines=False,
    horizontal_fov=360.0,  # Full rotation
    vertical_fov=30.0,     # 30° vertical spread
    horizontal_resolution=0.4,  # 0.4° per ray (900 rays)
    vertical_resolution=4.0,    # 32 beams (30/4 ≈ 8, but config handles this)
    rotation_rate=20.0,  # 20 Hz rotation
    high_lod=True,
    yaw_offset=0.0
)

# Initialize
world.reset()

# Simulate and capture point clouds
for i in range(50):
    world.step(render=True)

    # Get LIDAR data (every 10 steps)
    if i % 10 == 0:
        point_cloud_data = lidar_config.get_point_cloud_data(lidar_path)

        if point_cloud_data is not None and len(point_cloud_data) > 0:
            points = np.array(point_cloud_data)

            # Find closest obstacle
            distances = np.linalg.norm(points, axis=1)
            min_distance = distances.min()

            print(f"Step {i}: Point cloud has {len(points)} points")
            print(f"  Closest obstacle: {min_distance:.2f}m")

simulation_app.close()
```

**Output:**
- Visualizes LIDAR rays hitting obstacles
- Prints number of points and closest obstacle distance
- Returns 3D point cloud (N × 3 array)

---

## Python API: Programmatic Control

The Isaac Sim Python API allows full control over scenes, useful for:
- Automated testing
- Data generation pipelines
- Reinforcement learning environments
- Custom simulation workflows

### Common Operations

1. **Spawn/Delete Objects**
2. **Apply Forces/Torques**
3. **Get/Set Object Properties** (position, velocity, mass)
4. **Control Robot Joints**
5. **Trigger Physics Events**

---

### Example: Procedural Scene Generation

```python
"""
Isaac Sim Example: Procedural Scene
Generates 50 random cubes and simulates physics.
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
import random

# Create world
world = World()
world.scene.add_default_ground_plane()

# Generate 50 random cubes
cubes = []
for i in range(50):
    x = random.uniform(-5, 5)
    y = random.uniform(-5, 5)
    z = random.uniform(1, 3)  # Drop from height
    size = random.uniform(0.1, 0.5)

    cube = world.scene.add(
        DynamicCuboid(
            prim_path=f"/World/Cube_{i}",
            name=f"cube_{i}",
            position=[x, y, z],
            size=size,
            color=[random.random(), random.random(), random.random()]
        )
    )
    cubes.append(cube)

# Reset physics
world.reset()

# Simulate for 500 steps (≈25 seconds at 20 FPS)
for step in range(500):
    world.step(render=True)

    # Every 100 steps, apply random force to a cube
    if step % 100 == 0 and step > 0:
        random_cube = random.choice(cubes)
        force = [random.uniform(-50, 50), random.uniform(-50, 50), random.uniform(0, 100)]
        random_cube.apply_force(force)
        print(f"Step {step}: Applied force {force} to {random_cube.name}")

simulation_app.close()
```

**Output:**
- 50 cubes fall and collide with each other
- Every 100 steps, a random cube receives an impulse
- Realistic physics interactions (friction, collisions)

---

## Exporting Data for ML Pipelines

Isaac Sim supports **standard ML dataset formats** for seamless integration with training pipelines.

### Supported Formats

| Format | Use Case | File Structure |
|--------|----------|----------------|
| **COCO** | Object detection | JSON annotations + images |
| **YOLO** | Object detection | TXT files (normalized bbox) |
| **Pascal VOC** | Object detection | XML annotations |
| **KITTI** | Autonomous driving | Point clouds, images, calib |
| **Custom** | Any | Python writer API |

---

### Example: COCO Format Export

```python
"""
Isaac Sim Example: COCO Dataset Export
Generates 200 images with COCO annotations.
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni.replicator.core as rep
from omni.isaac.core import World

world = World()
world.scene.add_default_ground_plane()
world.reset()

# Create randomized scene
with rep.new_layer():
    # 5 different object types
    cubes = rep.create.cube(
        semantics=[("class", "cube")],
        count=5,
        position=rep.distribution.uniform((-2, -2, 0.5), (2, 2, 1.5)),
        scale=rep.distribution.uniform(0.2, 0.6),
        color=rep.distribution.uniform((0.5, 0, 0), (1, 0.5, 0.5))  # Red-ish
    )

    spheres = rep.create.sphere(
        semantics=[("class", "sphere")],
        count=5,
        position=rep.distribution.uniform((-2, -2, 0.5), (2, 2, 1.5)),
        scale=rep.distribution.uniform(0.2, 0.6),
        color=rep.distribution.uniform((0, 0.5, 0), (0.5, 1, 0.5))  # Green-ish
    )

    # Lighting
    lights = rep.create.light(
        light_type="Sphere",
        intensity=rep.distribution.uniform(2000, 6000),
        position=rep.distribution.uniform((-5, -5, 4), (5, 5, 10)),
        count=2
    )

    # Camera
    camera = rep.create.camera(
        position=rep.distribution.uniform((3, 3, 2), (5, 5, 4)),
        look_at=(0, 0, 0.5)
    )

    render_product = rep.create.render_product(camera, (640, 480))

    # COCO Writer
    writer = rep.WriterRegistry.get("COCOWriter")
    writer.initialize(
        output_dir="./coco_dataset",
        rgb=True,
        bounding_box_2d_tight=True,
        semantic_segmentation=True,
        instance_id_segmentation=True
    )
    writer.attach([render_product])

# Generate 200 frames
with rep.trigger.on_frame(num_frames=200):
    rep.randomizer.scatter_2d(
        [cubes, spheres],
        surface_prims="/World/defaultGroundPlane/geom"
    )

rep.orchestrator.run()

print("Dataset generated in ./coco_dataset/")
print("Format: COCO (compatible with Detectron2, MMDetection, etc.)")
simulation_app.close()
```

**Output Directory Structure:**
```
coco_dataset/
├── annotations/
│   └── instances.json          # COCO format annotations
├── images/
│   ├── 000000.png
│   ├── 000001.png
│   └── ... (200 images)
├── semantic_segmentation/
│   ├── 000000.png
│   └── ...
└── instance_segmentation/
    ├── 000000.png
    └── ...
```

**Using with PyTorch:**
```python
from pycocotools.coco import COCO

# Load annotations
coco = COCO('./coco_dataset/annotations/instances.json')

# Get all images with 'cube' class
cat_ids = coco.getCatIds(catNms=['cube'])
img_ids = coco.getImgIds(catIds=cat_ids)

print(f"Found {len(img_ids)} images with cubes")

# Load first image
img_info = coco.loadImgs(img_ids[0])[0]
ann_ids = coco.getAnnIds(imgIds=img_info['id'])
anns = coco.loadAnns(ann_ids)

print(f"Image: {img_info['file_name']}, Annotations: {len(anns)}")
```

---

## Key Takeaways

1. **Isaac Sim enables photorealistic simulation** with GPU-accelerated physics (PhysX) and ray-traced rendering (RTX)

2. **Synthetic data generation solves the data scarcity problem** — generate millions of labeled images automatically instead of manual labeling

3. **Domain randomization** (lighting, textures, poses, camera parameters) improves sim-to-real transfer by forcing models to learn robust features

4. **Python API provides full programmatic control** for automation, RL environments, and custom workflows

5. **Sensor simulation is realistic** — cameras, LIDAR, IMU with accurate noise models and physical properties

6. **Data export supports standard ML formats** (COCO, YOLO, KITTI) for seamless integration with training pipelines

7. **Hardware requirements are significant** — RTX 3060+ GPU with 12GB+ VRAM for good performance

8. **Isaac Sim integrates with ROS 2** (covered in Week 9) for robot testing and deployment

---

## Exercises

### Conceptual (2 questions)

**1. Sim-to-Real Gap**

Why does a model trained only on synthetic data often fail on real-world images? List **3 domain randomization strategies** that help bridge this gap.

<details>
<summary>Hint</summary>

Think about differences between simulation and reality: lighting, textures, sensor characteristics. How can randomization force the model to be robust to these variations?
</details>

---

**2. Gazebo vs Isaac Sim Use Cases**

You need to:
- (A) Test a navigation algorithm with basic obstacle avoidance
- (B) Train an object detection model for grasping

Which simulator(s) would you use for each task, and why?

<details>
<summary>Answer</summary>

- **(A) Gazebo**: Navigation testing doesn't require photorealism; Gazebo's CPU-based physics and ROS 2 integration are sufficient and faster to set up.
- **(B) Isaac Sim**: Training vision models requires diverse, realistic data. Isaac Sim's photorealistic rendering, domain randomization, and automatic labeling make it ideal for generating training datasets.
</details>

---

### Coding (3 exercises)

**3. Basic Scene with RGB + Depth**

Write an Isaac Sim Python script that:
1. Creates 5 spheres at random positions (x, y ∈ [-2, 2], z ∈ [0.5, 2])
2. Adds a camera at position (3, 3, 2) looking at origin
3. Simulates physics for 100 steps
4. Saves RGB and depth images every 10 frames (10 images total)

**Expected files:** `rgb_000.png`, `rgb_010.png`, ..., `depth_000.png`, `depth_010.png`, ...

<details>
<summary>Starter Code</summary>

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicSphere
from omni.isaac.sensor import Camera
import numpy as np
import matplotlib.pyplot as plt

world = World()
world.scene.add_default_ground_plane()

# TODO: Create 5 spheres at random positions

# TODO: Create camera

world.reset()

for i in range(100):
    world.step(render=True)

    # TODO: Save images every 10 frames

simulation_app.close()
```
</details>

---

**4. Domain Randomization**

Extend Exercise 3 to randomize:
1. Sphere colors (random RGB)
2. Lighting intensity (1000-5000 lumens)
3. Camera position (x, y ∈ [2, 4], z ∈ [1.5, 3])

Generate 50 randomized frames and save as COCO dataset.

<details>
<summary>Hint</summary>

Use `omni.replicator.core` with `rep.create`, `rep.distribution.uniform`, and `COCOWriter`.
</details>

---

**5. LIDAR Obstacle Detection**

Create a scene with:
1. 10 cubes at random positions (obstacles)
2. A 360° LIDAR sensor (32 beams, 0.4° resolution)
3. Simulate for 100 steps
4. Print the **minimum distance** to any obstacle every 10 steps

**Expected output:**
```
Step 0: Min distance = 2.34m
Step 10: Min distance = 2.31m
Step 20: Min distance = 2.28m
...
```

<details>
<summary>Hint</summary>

Use `_range_sensor.acquire_lidar_sensor_interface()` and `get_point_cloud_data()`. Compute distances with `np.linalg.norm(points, axis=1)`.
</details>

---

## Further Resources

**Official Documentation:**
- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/) - Complete reference
- [Replicator Tutorials](https://docs.omniverse.nvidia.com/prod_extensions/prod_extensions/ext_replicator.html) - Synthetic data generation
- [Python API Reference](https://docs.omniverse.nvidia.com/py/isaacsim/index.html) - All classes and functions

**Video Tutorials:**
- [NVIDIA Isaac Sim YouTube Channel](https://www.youtube.com/c/NVIDIAOmniverse) - Official tutorials
- "Getting Started with Isaac Sim" - Beginner walkthrough
- "Synthetic Data Generation for Robotics" - Domain randomization deep dive

**Research Papers:**
- *Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World* (2017) - Original DR paper
- *Sim-to-Real Transfer in Deep Reinforcement Learning for Robotics: a Survey* (2020) - Survey of techniques

**Community:**
- [NVIDIA Omniverse Forums](https://forums.developer.nvidia.com/c/omniverse/) - Ask questions
- [Isaac Sim GitHub Examples](https://github.com/NVIDIA-Omniverse/IsaacSim-ros_workspaces) - Sample code

---

**Next:** [Week 9: Isaac ROS Integration](./week9-isaac-ros-integration) - GPU-accelerated perception with Isaac ROS packages (cuVSLAM, DOPE, AprilTag detection).
