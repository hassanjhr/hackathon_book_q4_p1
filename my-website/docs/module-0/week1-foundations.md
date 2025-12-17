---
sidebar_position: 1
title: Week 1 - Foundations of Physical AI
---

# Week 1: Foundations of Physical AI

## What is Physical AI?

Physical AI refers to artificial intelligence systems that are **embodied** in physical forms—robots, autonomous vehicles, drones, and humanoid systems. Unlike traditional AI that operates purely in digital environments (image classification, language models, game-playing), Physical AI must:

1. **Perceive** the physical world through sensors
2. **Reason** about physical constraints and dynamics
3. **Act** through actuators that apply forces and motion
4. **Adapt** to real-time changes in unpredictable environments

### The Sense-Think-Act Loop

Every robotic system operates on a continuous feedback loop:

```
┌─────────┐      ┌─────────┐      ┌─────────┐
│ SENSE   │ ───> │ THINK   │ ───> │  ACT    │
│ (Sensors)│     │ (AI/ML) │     │(Actuators)│
└─────────┘      └─────────┘      └─────────┘
     ▲                                   │
     └───────────────────────────────────┘
              Feedback Loop
```

**Example:**
- **Sense**: Camera detects a cup on a table
- **Think**: AI determines grasp pose and trajectory
- **Act**: Robot arm moves to grasp the cup
- **Feedback**: Force sensors confirm successful grasp

## Digital AI vs Physical AI

| Aspect | Digital AI | Physical AI |
|--------|-----------|-------------|
| **Environment** | Virtual, simulated | Real-world, physical |
| **Consequences** | No physical risk | Safety-critical |
| **Timing** | Batch processing OK | Real-time required (< 100ms) |
| **Uncertainty** | Controlled inputs | Sensor noise, dynamics |
| **Validation** | Accuracy metrics | Safety + performance |
| **Errors** | Re-run possible | Irreversible damage |

### Example: Image Classification vs Robotic Grasping

**Digital AI (Image Classifier):**
```python
# Digital AI: No physical constraints
import torch
from torchvision import models, transforms

# Load model
model = models.resnet50(pretrained=True)
model.eval()

# Process image (can take seconds, no problem)
image = load_image("cup.jpg")
prediction = model(image)

# Wrong prediction? Just re-run
print(f"Predicted: {prediction}")
```

**Physical AI (Robot Grasping):**
```python
# Physical AI: Real-time + safety constraints
import rclpy
from geometry_msgs.msg import Pose

class GraspPlanner:
    def __init__(self):
        self.max_force = 50.0  # Newtons (safety limit)
        self.timeout = 0.1     # 100ms real-time constraint

    def plan_grasp(self, object_pose):
        """
        Must consider:
        - Physics: Will grasp succeed given friction?
        - Safety: Force limits to avoid crushing
        - Real-time: Decision within 100ms
        - Uncertainty: Object might slip
        """
        # Check physical constraints
        if not self.is_reachable(object_pose):
            return None  # Cannot violate kinematic limits

        # Plan considering dynamics
        grasp_pose = self.compute_stable_grasp(
            object_pose,
            friction_coef=0.6,  # Physical property
            max_force=self.max_force
        )

        return grasp_pose
```

## Physical Laws Awareness in Robotics

Physical AI systems must explicitly model and respect the laws of physics. Here are the key physical principles:

### 1. Newton's Laws of Motion

**First Law (Inertia):**
- Objects resist changes in motion
- Heavier robots require more force to accelerate
- Implications: Plan smooth trajectories to minimize energy

**Second Law (F = ma):**
- Force equals mass times acceleration
- Robots must compute required torques for desired motion
- Implications: Joint motors have torque limits

**Third Law (Action-Reaction):**
- Every action has an equal and opposite reaction
- When a robot pushes, it experiences pushback
- Implications: Balance control in bipedal robots

### 2. Dynamics vs Kinematics

**Kinematics** (position, velocity):
```python
# Forward kinematics: Joint angles → End effector position
def forward_kinematics(joint_angles):
    """
    Given joint angles [θ1, θ2, θ3, ...],
    compute end-effector position [x, y, z]
    """
    x = l1 * cos(θ1) + l2 * cos(θ1 + θ2)
    y = l1 * sin(θ1) + l2 * sin(θ1 + θ2)
    return (x, y)
```

**Dynamics** (forces, torques):
```python
# Inverse dynamics: Desired motion → Required torques
def inverse_dynamics(joint_angles, velocities, accelerations):
    """
    Given desired motion, compute torques needed
    considering inertia, gravity, friction
    """
    # Mass matrix (inertia effects)
    M = compute_mass_matrix(joint_angles)

    # Coriolis and centrifugal forces
    C = compute_coriolis(joint_angles, velocities)

    # Gravity compensation
    G = compute_gravity(joint_angles)

    # Required torques
    tau = M @ accelerations + C + G
    return tau
```

### 3. Energy and Efficiency

Robots have limited energy (battery life). Efficient motion planning is critical:

```python
# Energy-optimal trajectory planning
def plan_energy_efficient_trajectory(start, goal):
    """
    Minimize energy consumption while reaching goal
    """
    # Avoid high accelerations (quadratic energy cost)
    # Use gravity to assist motion
    # Smooth velocity profiles

    trajectory = optimize(
        cost_function=lambda path: compute_energy(path),
        constraints=[
            kinematic_limits,
            collision_free,
            time_limit
        ]
    )
    return trajectory
```

## Practical Example: Ball Catching Robot

Let's see how physical laws awareness applies to a real task:

**Problem:** Catch a thrown ball with a robot arm

**Physical considerations:**

1. **Trajectory prediction** (kinematics + gravity):
```python
def predict_ball_trajectory(initial_position, initial_velocity):
    """
    Predict where the ball will be using physics
    """
    g = 9.81  # Gravity (m/s²)
    t = 0.0   # Time
    dt = 0.01 # Time step

    position = initial_position
    velocity = initial_velocity

    trajectory = []
    while position[2] > 0:  # Until ball hits ground
        # Update using physics equations
        velocity[2] -= g * dt  # Gravity affects vertical velocity
        position += velocity * dt
        trajectory.append(position.copy())
        t += dt

    return trajectory
```

2. **Intercept planning** (inverse kinematics):
```python
def plan_intercept(ball_trajectory, robot_arm):
    """
    Find joint angles to position end-effector at intercept point
    """
    # Find intercept point (where robot can reach in time)
    intercept_point = None
    intercept_time = None

    for t, ball_pos in enumerate(ball_trajectory):
        # Can robot reach this point in time t?
        joint_angles = inverse_kinematics(ball_pos)
        motion_time = compute_motion_time(robot_arm, joint_angles)

        if motion_time <= t * 0.01:  # Feasible intercept
            intercept_point = ball_pos
            intercept_time = t * 0.01
            break

    return intercept_point, intercept_time
```

3. **Execution** (dynamics and control):
```python
def execute_catch(target_angles, current_angles):
    """
    Execute motion with real-time control
    """
    # Generate smooth trajectory
    trajectory = generate_trajectory(current_angles, target_angles)

    # Execute with feedback control
    for target_angle in trajectory:
        # Measure current state
        current = read_joint_angles()

        # PID control (react to errors)
        error = target_angle - current
        torque = Kp * error + Kd * derivative(error)

        # Apply torque (respecting limits)
        apply_torque(min(torque, MAX_TORQUE))

        sleep(0.01)  # 100 Hz control loop
```

## Key Takeaways

1. **Physical AI is fundamentally different from Digital AI** due to real-world constraints
2. **Real-time performance is critical** - decisions must be made in milliseconds
3. **Physics is not optional** - robots must respect Newton's laws, dynamics, energy limits
4. **Safety is paramount** - errors can cause damage or harm
5. **Uncertainty is unavoidable** - sensors are noisy, environments are unpredictable

## Exercises

1. **Conceptual**: Explain why a robot cannot instantly stop moving, even with perfect control
2. **Calculation**: Given a 5kg robot arm, compute the torque needed to lift it horizontally against gravity (assume 0.5m center of mass distance)
3. **Coding**: Simulate a 1D ball drop with gravity and predict landing time

## Next: Week 2 - Sensors and Humanoid Landscape

In Week 2, we'll explore how robots perceive the physical world through sensors and survey the current state of humanoid robotics.

---

**Further Reading:**
- "Probabilistic Robotics" by Thrun, Burgard, Fox
- "Modern Robotics" by Lynch and Park
- "Robot Dynamics and Control" by Spong, Hutchinson, Vidyasagar
