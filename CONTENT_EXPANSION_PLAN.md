# Content Expansion Plan: Physical AI & Humanoid Robotics Textbook

**Date:** 2025-12-16
**Objective:** Integrate weekly breakdown (Weeks 1-13) into existing Docusaurus structure

---

## Current Structure (Before Expansion)

```
docs/
├── intro.md (Welcome page)
├── module-1/ (ROS 2 Fundamentals)
│   └── intro.md
├── module-2/ (Digital Twin - Gazebo & Unity)
│   └── intro.md
├── module-3/ (AI-Robot Brain - NVIDIA Isaac)
│   └── intro.md
└── module-4/ (Vision-Language-Action)
    └── intro.md
```

---

## Proposed New Structure (After Expansion)

```
docs/
├── intro.md (Updated welcome)
│
├── module-0/ (NEW - Weeks 1-2)
│   ├── intro.md ✅ CREATED
│   ├── week1-foundations.md ✅ CREATED
│   └── week2-sensors-humanoids.md ✅ CREATED
│
├── module-1/ (Weeks 3-5 - ROS 2 Fundamentals)
│   ├── intro.md (existing - update to reference weeks)
│   ├── week3-ros2-architecture.md ✅ CREATED
│   ├── week4-launch-parameters.md 🔄 TO CREATE
│   └── week5-ros2-packages.md 🔄 TO CREATE
│
├── module-2/ (Weeks 6-7 - Robot Simulation)
│   ├── intro.md (existing - update to reference weeks)
│   ├── week6-gazebo-fundamentals.md 🔄 TO CREATE
│   └── week7-unity-visualization.md 🔄 TO CREATE
│
├── module-3/ (Weeks 8-10 - NVIDIA Isaac Platform)
│   ├── intro.md (existing - update to reference weeks)
│   ├── week8-isaac-sim-setup.md 🔄 TO CREATE
│   ├── week9-isaac-ros-perception.md 🔄 TO CREATE
│   └── week10-reinforcement-learning.md 🔄 TO CREATE
│
├── module-4/ (Week 13 - Conversational Robotics - EXPANDED)
│   ├── intro.md (existing - update to include conversational AI)
│   ├── week13-conversational-ai.md 🔄 TO CREATE
│   ├── llm-integration.md 🔄 TO CREATE
│   └── multimodal-interaction.md 🔄 TO CREATE
│
└── module-5/ (NEW - Weeks 11-12 - Humanoid Robotics)
    ├── intro.md 🔄 TO CREATE
    ├── week11-kinematics-dynamics.md 🔄 TO CREATE
    └── week12-bipedal-locomotion.md 🔄 TO CREATE
```

---

## Weekly Content Mapping

### ✅ Module 0: Introduction to Physical AI (Weeks 1-2) - COMPLETE

**Week 1: Foundations**
- [x] Physical AI vs Digital AI explanation
- [x] Sense-think-act loop with examples
- [x] Physical laws awareness (Newton's laws, dynamics, kinematics)
- [x] Python code examples (ball catching robot)

**Week 2: Sensors & Humanoid Landscape**
- [x] LIDAR (theory + Python example)
- [x] Cameras (RGB, depth, stereo + OpenCV example)
- [x] IMUs (accelerometer, gyroscope + sensor fusion)
- [x] Force/Torque sensors (grasp control example)
- [x] Humanoid robotics overview (Atlas, Optimus, Figure 01, Digit)

---

### 🔄 Module 1: ROS 2 Fundamentals (Weeks 3-5) - IN PROGRESS

**Week 3: ROS 2 Architecture & Nodes** ✅ COMPLETE
- [x] ROS 2 graph architecture
- [x] Nodes, topics, services, actions
- [x] Python publisher/subscriber examples
- [x] Service client/server examples
- [x] Action server example (Fibonacci)

**Week 4: Launch Files & Parameters** 🔄 TO CREATE
- [ ] Launch file syntax and structure
- [ ] Multi-node launch examples
- [ ] Parameter declaration and usage
- [ ] YAML configuration files
- [ ] Namespaces and remapping

**Week 5: ROS 2 Packages & Best Practices** 🔄 TO CREATE
- [ ] Package structure (ament_python, ament_cmake)
- [ ] Dependencies and package.xml
- [ ] Building with colcon
- [ ] Testing and debugging tools
- [ ] Publisher/subscriber comparison table

---

### 🔄 Module 2: Robot Simulation (Weeks 6-7) - TO CREATE

**Week 6: Gazebo Fundamentals** 🔄 TO CREATE
- [ ] Gazebo environment setup
- [ ] URDF (Unified Robot Description Format)
- [ ] SDF (Simulation Description Format)
- [ ] Physics engines (ODE, Bullet, Simbody)
- [ ] Sensor simulation (camera, LIDAR in Gazebo)
- [ ] URDF/SDF code examples

**Week 7: Unity Visualization** 🔄 TO CREATE
- [ ] Unity setup for robotics
- [ ] ROS-TCP-Connector integration
- [ ] Real-time visualization workflows
- [ ] Humanoid animation in Unity
- [ ] Digital twin workflow comparison (Gazebo vs Unity)

---

### 🔄 Module 3: NVIDIA Isaac Platform (Weeks 8-10) - TO CREATE

**Week 8: Isaac Sim Setup & SDK** 🔄 TO CREATE
- [ ] Isaac Sim installation and configuration
- [ ] Isaac SDK overview
- [ ] Synthetic data generation
- [ ] Camera and sensor simulation in Isaac
- [ ] Python API examples

**Week 9: Isaac ROS for AI Perception** 🔄 TO CREATE
- [ ] Isaac ROS packages overview
- [ ] AI-based perception pipelines
- [ ] Object detection and tracking
- [ ] Integration with ROS 2 nodes
- [ ] Perception-to-action flow diagram

**Week 10: Reinforcement Learning Basics** 🔄 TO CREATE
- [ ] RL fundamentals (states, actions, rewards)
- [ ] Sim-to-real transfer concepts
- [ ] Isaac Gym introduction
- [ ] Training a simple policy (reaching task)
- [ ] High-level RL pipeline explanation

---

### 🔄 Module 5: Humanoid Robotics (Weeks 11-12) - TO CREATE

**Week 11: Kinematics & Dynamics** 🔄 TO CREATE
- [ ] Forward kinematics (DH parameters)
- [ ] Inverse kinematics (analytical, numerical)
- [ ] Jacobians and velocity kinematics
- [ ] Dynamics equations (Lagrangian, Newton-Euler)
- [ ] Python examples with simple arm

**Week 12: Bipedal Locomotion & Balance** 🔄 TO CREATE
- [ ] Gait planning fundamentals
- [ ] Zero Moment Point (ZMP) criterion
- [ ] Balance control strategies
- [ ] Humanoid manipulation challenges
- [ ] Gait planning pseudo-code
- [ ] Balance control example

---

### 🔄 Module 4: Conversational Robotics (Week 13) - TO EXPAND

**Week 13: Conversational AI for Robots** 🔄 TO CREATE
- [ ] Conversational AI architecture
- [ ] LLM integration with ROS 2
- [ ] Speech recognition (Whisper integration)
- [ ] Intent parsing and command flow
- [ ] Multimodal interaction (voice + vision)
- [ ] Complete robot command flow example
- [ ] Safety and constraint handling

---

## Content Style Guidelines

### Implementation Style (Already Applied)

1. **Beginner-friendly but technical**
   - Explain concepts before showing code
   - Use analogies and diagrams
   - Provide context for why things matter

2. **Code-centric**
   - Python primary language
   - ROS 2 examples throughout
   - Runnable, self-contained examples
   - Comments explaining each section

3. **Self-contained**
   - No external dependencies for understanding
   - Include prerequisites at start
   - Link to related modules
   - Provide next steps

4. **Practical examples**
   - Real-world use cases
   - Sensor processing examples
   - Robot control scenarios
   - Debugging tips

### Format Standards

**Each chapter should include:**
- Learning objectives (bullet list)
- Conceptual explanation (text + diagrams)
- Code examples (Python, ROS 2)
- Key takeaways (numbered list)
- Exercises (3-5 questions)
- Next steps / links to next chapter

**Code blocks should:**
- Use syntax highlighting (```python)
- Include comments explaining logic
- Be runnable (or clearly marked as pseudo-code)
- Show expected output

---

## Files Created ✅

1. `/my-website/docs/module-0/intro.md` - Module 0 overview
2. `/my-website/docs/module-0/week1-foundations.md` - Physical AI foundations
3. `/my-website/docs/module-0/week2-sensors-humanoids.md` - Sensors & humanoid landscape
4. `/my-website/docs/module-1/week3-ros2-architecture.md` - ROS 2 architecture & nodes

---

## Files To Create 🔄

### High Priority (Core Learning Path)

1. **Module 1 (ROS 2):**
   - `week4-launch-parameters.md` - Launch files, parameters
   - `week5-ros2-packages.md` - Package structure, best practices

2. **Module 2 (Simulation):**
   - `week6-gazebo-fundamentals.md` - Gazebo, URDF, SDF
   - `week7-unity-visualization.md` - Unity integration

3. **Module 5 (Humanoid Robotics):**
   - `intro.md` - Module overview
   - `week11-kinematics-dynamics.md` - Kinematics and dynamics
   - `week12-bipedal-locomotion.md` - Gait planning, balance

4. **Module 4 (Conversational AI):**
   - `week13-conversational-ai.md` - LLM integration, speech

### Medium Priority (Advanced Topics)

5. **Module 3 (Isaac Platform):**
   - `week8-isaac-sim-setup.md` - Isaac Sim basics
   - `week9-isaac-ros-perception.md` - Perception pipelines
   - `week10-reinforcement-learning.md` - RL fundamentals

---

## Navigation Updates Required

### Update `sidebars.ts`

Currently using auto-generated sidebars. Should remain compatible.

### Update `docusaurus.config.ts`

Update navbar links to include Module 0 and Module 5:

```typescript
items: [
  {
    to: '/docs/module-0/intro',
    label: 'Module 0',
    position: 'left',
  },
  // ... existing Module 1-4 links
  {
    to: '/docs/module-5/intro',
    label: 'Module 5',
    position: 'left',
  },
]
```

Update footer links:
```typescript
{
  title: 'Course Modules',
  items: [
    { label: 'Module 0: Intro to Physical AI', to: '/docs/module-0/intro' },
    { label: 'Module 1: ROS 2 Fundamentals', to: '/docs/module-1/intro' },
    // ... etc
    { label: 'Module 5: Humanoid Robotics', to: '/docs/module-5/intro' },
  ],
},
```

---

## Testing Checklist

- [ ] Build succeeds: `npm run build` (in my-website/)
- [ ] All links work (no broken internal links)
- [ ] Sidebar navigation is logical
- [ ] Code blocks render correctly
- [ ] Images/diagrams display (if added)
- [ ] Mobile responsive layout works
- [ ] Search functionality includes new content

---

## Summary

**Total Weeks:** 13
**Total Modules:** 6 (0-5)
**Files Created:** 4 ✅
**Files To Create:** ~15 🔄

**Next Immediate Steps:**
1. Create Week 4 & 5 for Module 1 (complete ROS 2 content)
2. Create Module 5 (Humanoid Robotics - critical missing module)
3. Create Week 6 & 7 for Module 2 (Gazebo & Unity details)
4. Expand Module 4 with Week 13 conversational AI content
5. Update navigation configuration
6. Test build and validate all links

