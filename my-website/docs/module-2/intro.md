# Module 2: The Digital Twin (Gazebo & Unity)

Welcome to Module 2, where you'll master simulation environments and digital twin workflows for safe robot testing.

**Duration:** 2 weeks (Weeks 6-7)

## What You'll Build

By the end of this module, you will have:
- 🤖 **A complete robot simulation** in Gazebo with sensors (camera, LIDAR, IMU)
- 🎮 **A Unity teleoperation dashboard** with live camera feed and controls
- 🌍 **Custom simulation environments** for testing robot algorithms
- 📊 **Digital twin visualization** that mirrors real robot state

## What You'll Learn

In this module, you'll explore:

- **Week 6: Gazebo Fundamentals** - URDF robot models, SDF worlds, physics simulation, sensor integration
- **Week 7: Unity Visualization** - ROS 2 ↔ Unity communication, interactive UIs, humanoid animation

## Prerequisites

Before starting this module, you should have:

- ✅ Completed Module 1: ROS 2 Fundamentals
- ✅ Basic understanding of ROS 2 nodes, topics, and services
- ✅ Installed Gazebo Classic with ROS 2 Humble
- ✅ Unity 2021.3+ LTS (for Week 7)

## Weekly Roadmap

### [Week 6: Gazebo Fundamentals](./week6-gazebo-fundamentals)

**Learning Objectives:**
- Understand why simulation is critical for robotics development
- Create robot models using URDF (Unified Robot Description Format)
- Build simulation worlds with SDF (Simulation Description Format)
- Integrate sensors (camera, LIDAR, IMU) in Gazebo
- Launch and control robots in Gazebo from ROS 2

**Topics Covered:**
- Why simulate? (Cost, safety, iteration speed)
- URDF structure (links, joints, inertia tensors)
- SDF world files (physics, lighting, obstacles)
- Sensor plugins (camera, LIDAR, IMU)
- Gazebo-ROS 2 integration
- Mobile robots with differential drive

**Key Deliverable:** 2-link robot arm and mobile robot in Gazebo

---

### [Week 7: Unity Visualization](./week7-unity-visualization)

**Learning Objectives:**
- Understand when to use Unity vs Gazebo for robotics
- Set up ROS 2 ↔ Unity communication via ROS-TCP-Connector
- Import and control robot models in Unity
- Visualize sensor data (camera, LIDAR) in Unity
- Build interactive UIs for robot control

**Topics Covered:**
- Gazebo vs Unity comparison (physics vs visualization)
- ROS-TCP-Connector setup (Unity + ROS 2)
- Coordinate system conversion (ROS Z-up → Unity Y-up)
- Articulation Bodies for robot joints
- Publishing and subscribing between Unity and ROS 2
- Building teleoperation dashboards
- Humanoid animation with Unity Animator

**Key Deliverable:** Teleoperation dashboard with camera feed and joystick control

---

## Learning Outcomes

By the end of this module, you will be able to:

1. 🎯 Launch and configure Gazebo simulations with custom physics parameters
2. 🌍 Build custom simulation environments using URDF and SDF
3. 🤖 Integrate sensors (camera, LIDAR, IMU) in simulation
4. 🎮 Create Unity visualizations with ROS 2 integration
5. 📊 Build teleoperation dashboards with live sensor feeds
6. ✅ Validate robot behaviors in simulation before hardware deployment

## Module Structure

```
Week 6: Physics-Based Simulation (Gazebo)
  ↓
  • Robot modeling (URDF)
  • World creation (SDF)
  • Sensor simulation
  ↓
Week 7: Visualization & Interaction (Unity)
  ↓
  • ROS-Unity bridge
  • Interactive UIs
  • Humanoid animation
  ↓
Ready for Module 3: NVIDIA Isaac Sim (Advanced simulation + AI)
```

## Getting Started

Ready to dive into simulation? Let's begin with **Week 6: Gazebo Fundamentals**!

---

**Next**: [Week 6: Gazebo Fundamentals](./week6-gazebo-fundamentals)
