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
- Train a basic policy with Stable-Baselines3

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
