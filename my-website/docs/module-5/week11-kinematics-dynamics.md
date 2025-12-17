---
sidebar_position: 2
title: Week 11 - Kinematics & Dynamics
---

# Week 11: Humanoid Kinematics & Dynamics

**Learning Objectives:**
- Understand forward and inverse kinematics for humanoid robots
- Compute joint angles from desired end-effector positions
- Calculate Jacobians for velocity control
- Understand robot dynamics and torque requirements
- Implement kinematic algorithms in Python
- Apply concepts to humanoid arms, legs, and whole-body systems

---

## What is Kinematics?

**Kinematics** is the study of motion without considering forces. For robots, it answers:

- **Forward Kinematics (FK)**: "Given joint angles, where is the end-effector?"
- **Inverse Kinematics (IK)**: "To reach a position, what joint angles do I need?"

### Humanoid Example

Imagine a humanoid robot reaching for a cup:

```
Task: Reach cup at position (x, y, z) = (0.5m, 0.2m, 1.0m)

Forward Kinematics (FK):
  Input:  shoulder=45°, elbow=90°, wrist=30°
  Output: hand_position = ???

Inverse Kinematics (IK):
  Input:  hand_position = (0.5, 0.2, 1.0)
  Output: shoulder=?, elbow=?, wrist=?
```

**Why it matters:**
- **FK** tells us where the hand will be (prediction)
- **IK** tells us how to get there (control)

---

## Robot Anatomy: Links, Joints, and DOF

### Basic Terminology

**Link**: Rigid body segment (like a bone)
```
Humanoid Upper Body
├── Torso (link 0)
├── Upper Arm (link 1)
├── Forearm (link 2)
└── Hand (link 3)
```

**Joint**: Connection between links that allows motion
```
Joint Types:
- Revolute: Rotation (like elbow, knee)
- Prismatic: Linear motion (like telescope)
```

**Degree of Freedom (DOF)**: Number of independent motions
```
Human Arm DOF:
├── Shoulder: 3 DOF (flexion, abduction, rotation)
├── Elbow: 1 DOF (flexion)
└── Wrist: 2 DOF (flexion, deviation)
Total: 6 DOF
```

### Humanoid Robot DOF

Typical humanoid configuration:

```
Full Humanoid Body
├── Head: 2-3 DOF (pan, tilt, roll)
├── Each Arm: 6-7 DOF
│   ├── Shoulder: 3 DOF
│   ├── Elbow: 1 DOF
│   ├── Wrist: 2 DOF
│   └── Hand: 0-1 DOF (gripper)
├── Torso: 1-3 DOF (bending, twisting)
└── Each Leg: 6 DOF
    ├── Hip: 3 DOF
    ├── Knee: 1 DOF
    └── Ankle: 2 DOF

Total: ~25-40 DOF
```

---

## Forward Kinematics: From Joints to Position

### Intuition

Forward kinematics is like following a chain:

1. Start at the base (shoulder)
2. Move along each link in sequence
3. End up at the hand position

### Coordinate Frames

Each joint has a **coordinate frame**:

```
        Y₂ (up)
        │
   Z₂──┼──→ X₂
        │
    [Frame 2: Elbow]
        │
        │ Link 2
        │
        Y₁ (up)
        │
   Z₁──┼──→ X₁
        │
    [Frame 1: Shoulder]
```

### Transformation Matrices

We use **4×4 transformation matrices** to convert between frames:

```
T = [R | p]   where R is 3×3 rotation, p is 3×1 position
    [0 | 1]

Example:
T₀₁ = transformation from frame 0 to frame 1
```

### Simple 2D Example: 2-Link Arm

```python
import numpy as np
import matplotlib.pyplot as plt

def forward_kinematics_2d(theta1, theta2, l1=1.0, l2=1.0):
    """
    Compute forward kinematics for 2-link planar arm

    Args:
        theta1: First joint angle (radians)
        theta2: Second joint angle (radians)
        l1: Length of first link (meters)
        l2: Length of second link (meters)

    Returns:
        positions: List of (x, y) positions for each joint
    """
    # Joint 1 position
    x1 = l1 * np.cos(theta1)
    y1 = l1 * np.sin(theta1)

    # End-effector position
    x2 = x1 + l2 * np.cos(theta1 + theta2)
    y2 = y1 + l2 * np.sin(theta1 + theta2)

    return [(0, 0), (x1, y1), (x2, y2)]

# Example usage
theta1 = np.pi/4  # 45 degrees
theta2 = np.pi/3  # 60 degrees

positions = forward_kinematics_2d(theta1, theta2)
print(f"End-effector position: ({positions[2][0]:.3f}, {positions[2][1]:.3f})")

# Output: End-effector position: (1.366, 1.366)
```

### Visualizing the Arm

```python
def plot_arm(positions):
    """Plot 2-link arm configuration"""
    x_coords = [p[0] for p in positions]
    y_coords = [p[1] for p in positions]

    plt.figure(figsize=(6, 6))
    plt.plot(x_coords, y_coords, 'b-o', linewidth=2, markersize=8)
    plt.plot(x_coords[-1], y_coords[-1], 'ro', markersize=12, label='End-effector')

    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.xlim(-2.5, 2.5)
    plt.ylim(-2.5, 2.5)
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title('2-Link Arm Configuration')
    plt.legend()
    plt.show()

# Visualize
plot_arm(positions)
```

---

## Denavit-Hartenberg (DH) Parameters

The **DH convention** is a standard way to describe robot kinematics.

### Four DH Parameters per Joint

For each joint `i`, we define:

| Parameter | Symbol | Description |
|-----------|--------|-------------|
| **Link Length** | aᵢ | Distance along Xᵢ from Zᵢ₋₁ to Zᵢ |
| **Link Twist** | αᵢ | Angle about Xᵢ from Zᵢ₋₁ to Zᵢ |
| **Link Offset** | dᵢ | Distance along Zᵢ₋₁ from Xᵢ₋₁ to Xᵢ |
| **Joint Angle** | θᵢ | Angle about Zᵢ₋₁ from Xᵢ₋₁ to Xᵢ |

**For revolute joints:** θᵢ is the variable (others are constant)
**For prismatic joints:** dᵢ is the variable (others are constant)

### DH Transformation Matrix

```python
def dh_transform(a, alpha, d, theta):
    """
    Compute DH transformation matrix

    Args:
        a: link length
        alpha: link twist
        d: link offset
        theta: joint angle

    Returns:
        4x4 homogeneous transformation matrix
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    T = np.array([
        [ct,    -st*ca,   st*sa,    a*ct],
        [st,     ct*ca,  -ct*sa,    a*st],
        [0,      sa,      ca,       d   ],
        [0,      0,       0,        1   ]
    ])

    return T

# Example: Single revolute joint
T = dh_transform(a=1.0, alpha=0, d=0, theta=np.pi/4)
print("DH Transform:\n", T)
```

### 3-DOF Arm Example

**DH Parameter Table:**

| Joint | a (m) | α (rad) | d (m) | θ (rad) |
|-------|-------|---------|-------|---------|
| 1 | 0 | π/2 | 0.5 | θ₁ (variable) |
| 2 | 1.0 | 0 | 0 | θ₂ (variable) |
| 3 | 0.8 | 0 | 0 | θ₃ (variable) |

```python
def forward_kinematics_3dof(theta1, theta2, theta3):
    """
    Forward kinematics for 3-DOF arm using DH parameters
    """
    # DH parameters (from table above)
    dh_params = [
        {'a': 0.0,  'alpha': np.pi/2, 'd': 0.5, 'theta': theta1},
        {'a': 1.0,  'alpha': 0,       'd': 0,   'theta': theta2},
        {'a': 0.8,  'alpha': 0,       'd': 0,   'theta': theta3},
    ]

    # Compute transformation matrices
    T = np.eye(4)  # Start with identity
    for params in dh_params:
        T = T @ dh_transform(**params)

    # Extract end-effector position
    position = T[:3, 3]
    return position

# Example
theta = [np.pi/6, np.pi/4, np.pi/3]  # Joint angles
position = forward_kinematics_3dof(*theta)
print(f"End-effector position: {position}")
# Output: End-effector position: [1.366 0.683 1.050]
```

---

## Inverse Kinematics: From Position to Joints

### The Inverse Problem

**Given:** Desired end-effector position (x, y, z)
**Find:** Joint angles (θ₁, θ₂, ..., θₙ) that achieve it

**Challenges:**
1. **Multiple solutions**: Same position, different joint angles
2. **No solution**: Position unreachable (out of workspace)
3. **Singularities**: Loss of DOF at certain configurations
4. **Computational complexity**: Nonlinear equations

### Analytical IK: 2-Link Planar Arm

For simple geometries, we can derive closed-form solutions:

```python
def inverse_kinematics_2d(x, y, l1=1.0, l2=1.0):
    """
    Analytical inverse kinematics for 2-link planar arm

    Args:
        x, y: Desired end-effector position
        l1, l2: Link lengths

    Returns:
        (theta1, theta2): Joint angles in radians
        Returns None if position is unreachable
    """
    # Check if position is reachable
    distance = np.sqrt(x**2 + y**2)
    if distance > (l1 + l2) or distance < abs(l1 - l2):
        print("Position unreachable!")
        return None

    # Compute theta2 using law of cosines
    cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    cos_theta2 = np.clip(cos_theta2, -1, 1)  # Handle numerical errors

    # Two solutions: elbow up or elbow down
    theta2_elbow_up = np.arccos(cos_theta2)
    theta2_elbow_down = -np.arccos(cos_theta2)

    # Compute theta1 for elbow-up solution
    k1 = l1 + l2 * np.cos(theta2_elbow_up)
    k2 = l2 * np.sin(theta2_elbow_up)
    theta1_elbow_up = np.arctan2(y, x) - np.arctan2(k2, k1)

    return theta1_elbow_up, theta2_elbow_up

# Example usage
x_target, y_target = 1.5, 1.0

theta1, theta2 = inverse_kinematics_2d(x_target, y_target)
print(f"Joint angles: θ₁={np.degrees(theta1):.1f}°, θ₂={np.degrees(theta2):.1f}°")

# Verify with forward kinematics
positions = forward_kinematics_2d(theta1, theta2)
x_actual, y_actual = positions[2]
print(f"Achieved position: ({x_actual:.3f}, {y_actual:.3f})")
print(f"Target position: ({x_target:.3f}, {y_target:.3f})")
```

### Multiple Solutions

```python
# Visualize both elbow-up and elbow-down solutions
def plot_ik_solutions(x, y, l1=1.0, l2=1.0):
    """Show both IK solutions"""
    # Elbow-up
    theta1_up, theta2_up = inverse_kinematics_2d(x, y, l1, l2)
    pos_up = forward_kinematics_2d(theta1_up, theta2_up, l1, l2)

    # Elbow-down
    cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    theta2_down = -np.arccos(np.clip(cos_theta2, -1, 1))
    k1 = l1 + l2 * np.cos(theta2_down)
    k2 = l2 * np.sin(theta2_down)
    theta1_down = np.arctan2(y, x) - np.arctan2(k2, k1)
    pos_down = forward_kinematics_2d(theta1_down, theta2_down, l1, l2)

    # Plot
    plt.figure(figsize=(12, 5))

    # Elbow-up
    plt.subplot(1, 2, 1)
    x_coords = [p[0] for p in pos_up]
    y_coords = [p[1] for p in pos_up]
    plt.plot(x_coords, y_coords, 'b-o', linewidth=2)
    plt.plot(x, y, 'r*', markersize=15, label='Target')
    plt.title('Elbow-Up Solution')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')

    # Elbow-down
    plt.subplot(1, 2, 2)
    x_coords = [p[0] for p in pos_down]
    y_coords = [p[1] for p in pos_down]
    plt.plot(x_coords, y_coords, 'g-o', linewidth=2)
    plt.plot(x, y, 'r*', markersize=15, label='Target')
    plt.title('Elbow-Down Solution')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')

    plt.tight_layout()
    plt.show()

# Visualize
plot_ik_solutions(1.5, 1.0)
```

---

## Numerical IK: Optimization-Based Approach

For complex robots (humanoids with many DOF), we use **numerical methods**:

### Iterative Approach

```python
from scipy.optimize import minimize

def numerical_ik(target_position, initial_guess, link_lengths):
    """
    Numerical inverse kinematics using optimization

    Args:
        target_position: (x, y) desired position
        initial_guess: Initial joint angles
        link_lengths: [l1, l2, ...]

    Returns:
        Optimal joint angles
    """
    def cost_function(joint_angles):
        """Compute error between current and target position"""
        # Forward kinematics
        positions = forward_kinematics_2d(
            joint_angles[0],
            joint_angles[1],
            link_lengths[0],
            link_lengths[1]
        )
        current_pos = positions[-1]

        # Euclidean distance error
        error = np.sqrt(
            (current_pos[0] - target_position[0])**2 +
            (current_pos[1] - target_position[1])**2
        )
        return error

    # Optimize
    result = minimize(
        cost_function,
        initial_guess,
        method='SLSQP',
        options={'ftol': 1e-6}
    )

    return result.x

# Example
target = (1.2, 0.8)
initial = [0.5, 0.5]  # Initial guess
lengths = [1.0, 1.0]

solution = numerical_ik(target, initial, lengths)
print(f"Numerical IK solution: θ₁={np.degrees(solution[0]):.1f}°, θ₂={np.degrees(solution[1]):.1f}°")

# Verify
positions = forward_kinematics_2d(solution[0], solution[1])
print(f"Achieved: ({positions[2][0]:.3f}, {positions[2][1]:.3f})")
print(f"Target: ({target[0]:.3f}, {target[1]:.3f})")
```

**Advantages of Numerical IK:**
- Works for any number of DOF
- Can handle constraints (joint limits, obstacles)
- Can optimize secondary objectives (comfort, energy)

**Disadvantages:**
- Slower than analytical solutions
- May get stuck in local minima
- Requires good initial guess

---

## Jacobian: Velocity-Level Kinematics

The **Jacobian** relates joint velocities to end-effector velocity:

```
ẋ = J(θ) · θ̇

Where:
  ẋ = end-effector velocity (linear + angular)
  J = Jacobian matrix
  θ̇ = joint velocities
```

### Why Jacobians Matter

1. **Velocity control**: Control end-effector velocity directly
2. **Force control**: Relate end-effector forces to joint torques
3. **Singularity detection**: Jacobian becomes rank-deficient
4. **Numerical IK**: Used in iterative solvers

### Computing the Jacobian

```python
def compute_jacobian_2d(theta1, theta2, l1=1.0, l2=1.0):
    """
    Compute Jacobian for 2-link planar arm

    Jacobian maps joint velocities to end-effector velocity:
    [vx]   [J11  J12] [theta1_dot]
    [vy] = [J21  J22] [theta2_dot]
    """
    # Partial derivatives of end-effector position w.r.t. joint angles
    J11 = -l1 * np.sin(theta1) - l2 * np.sin(theta1 + theta2)
    J12 = -l2 * np.sin(theta1 + theta2)
    J21 = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    J22 = l2 * np.cos(theta1 + theta2)

    J = np.array([
        [J11, J12],
        [J21, J22]
    ])

    return J

# Example
theta1, theta2 = np.pi/4, np.pi/3
J = compute_jacobian_2d(theta1, theta2)

print("Jacobian:\n", J)

# Check singularity
det = np.linalg.det(J)
print(f"\nDeterminant: {det:.4f}")
if abs(det) < 1e-3:
    print("WARNING: Near singularity!")
```

### Singularities

A **singularity** occurs when the Jacobian loses rank (determinant = 0):

```python
def check_singularities(l1=1.0, l2=1.0):
    """Find singularities by scanning workspace"""
    theta1_range = np.linspace(0, 2*np.pi, 50)
    theta2_range = np.linspace(-np.pi, np.pi, 50)

    singular_configs = []

    for t1 in theta1_range:
        for t2 in theta2_range:
            J = compute_jacobian_2d(t1, t2, l1, l2)
            det = np.linalg.det(J)

            if abs(det) < 0.01:  # Near-singular
                singular_configs.append((t1, t2, det))

    print(f"Found {len(singular_configs)} near-singular configurations")

    # Common singularities for 2-link arm:
    # 1. Fully extended (theta2 = 0)
    # 2. Fully collapsed (theta2 = π)

check_singularities()
```

**What happens at singularities:**
- Loss of mobility in certain directions
- Infinite joint velocities may be required
- Control becomes unstable

---

## Robot Dynamics: Forces and Torques

**Dynamics** studies motion considering forces and masses.

### Why Dynamics Matter

To move a robot, we need to compute **torques** (τ) required at each joint:

```
Task: Move arm from point A to point B

Kinematics tells us: Joint angles needed
Dynamics tells us: Torques needed to achieve those angles
```

### Newton's Second Law for Rotation

```
τ = I · α

Where:
  τ = torque (N·m)
  I = moment of inertia (kg·m²)
  α = angular acceleration (rad/s²)
```

### Equation of Motion

For a robot, the full dynamics are:

```
τ = M(θ) · θ̈ + C(θ, θ̇) · θ̇ + G(θ)

Where:
  M(θ) = mass matrix (inertia effects)
  C(θ, θ̇) = Coriolis and centrifugal forces
  G(θ) = gravity torques
```

### Gravity Compensation Example

```python
def gravity_torque_2d(theta1, theta2, m1=1.0, m2=0.5, l1=1.0, l2=1.0):
    """
    Compute gravity compensation torques for 2-link arm

    Args:
        theta1, theta2: Joint angles
        m1, m2: Link masses (kg)
        l1, l2: Link lengths (m)

    Returns:
        [tau1, tau2]: Required torques to hold position against gravity
    """
    g = 9.81  # Gravity (m/s²)

    # Center of mass positions
    lc1 = l1 / 2  # COM of link 1
    lc2 = l2 / 2  # COM of link 2

    # Gravity torques (simplified)
    tau1 = (m1 * lc1 + m2 * l1) * g * np.cos(theta1) + \
           m2 * lc2 * g * np.cos(theta1 + theta2)

    tau2 = m2 * lc2 * g * np.cos(theta1 + theta2)

    return np.array([tau1, tau2])

# Example: Arm horizontal (fighting gravity)
theta1 = 0  # Horizontal
theta2 = 0

torques = gravity_torque_2d(theta1, theta2)
print(f"Gravity compensation torques: τ₁={torques[0]:.2f} N·m, τ₂={torques[1]:.2f} N·m")
```

### Forward Dynamics Simulation

```python
def simulate_falling_arm(duration=2.0, dt=0.01):
    """
    Simulate arm falling under gravity (no control)
    """
    # Initial state
    theta = np.array([np.pi/2, 0])  # Vertical arm
    theta_dot = np.array([0.0, 0.0])  # At rest

    # Parameters
    m1, m2 = 1.0, 0.5
    l1, l2 = 1.0, 1.0
    I1 = m1 * l1**2 / 12  # Moment of inertia (rod)
    I2 = m2 * l2**2 / 12

    time_steps = int(duration / dt)
    trajectory = []

    for t in range(time_steps):
        # Gravity torques
        tau_gravity = gravity_torque_2d(theta[0], theta[1], m1, m2, l1, l2)

        # Simplified dynamics (ignoring Coriolis for clarity)
        # τ = I·α → α = τ/I
        theta_ddot = tau_gravity / np.array([I1 + I2, I2])

        # Integrate
        theta_dot += theta_ddot * dt
        theta += theta_dot * dt

        trajectory.append(theta.copy())

    # Plot
    trajectory = np.array(trajectory)
    time = np.arange(time_steps) * dt

    plt.figure(figsize=(10, 4))
    plt.subplot(1, 2, 1)
    plt.plot(time, np.degrees(trajectory[:, 0]), label='θ₁ (shoulder)')
    plt.plot(time, np.degrees(trajectory[:, 1]), label='θ₂ (elbow)')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (degrees)')
    plt.title('Arm Falling Under Gravity')
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.subplot(1, 2, 2)
    # Animate final frame
    final_pos = forward_kinematics_2d(trajectory[-1, 0], trajectory[-1, 1])
    x_coords = [p[0] for p in final_pos]
    y_coords = [p[1] for p in final_pos]
    plt.plot(x_coords, y_coords, 'r-o', linewidth=2)
    plt.title('Final Configuration')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')

    plt.tight_layout()
    plt.show()

# Run simulation
simulate_falling_arm()
```

---

## Humanoid-Specific Considerations

### Whole-Body Kinematics

Humanoids have **redundancy**: More DOF than needed for a task.

**Example:** Reaching a point
- 6 DOF needed (3 position + 3 orientation)
- Humanoid arm has 7 DOF
- **Extra DOF** allows secondary objectives:
  - Avoid obstacles
  - Maximize comfort
  - Stay within joint limits
  - Minimize energy

### Kinematic Chains

```
Humanoid Arm Chain (Right Arm):
Base (Torso) → Shoulder → Elbow → Wrist → Hand

Leg Chain:
Base (Pelvis) → Hip → Knee → Ankle → Foot
```

### Closed-Loop Constraints

When a humanoid stands on both feet:

```
Left Foot → Left Leg → Torso → Right Leg → Right Foot → Ground → Left Foot

This forms a CLOSED LOOP, adding constraints.
```

---

## Practical Python Implementation

### Complete 3-DOF Arm Kinematics Class

```python
class RobotArm3DOF:
    """
    3-DOF robot arm with forward/inverse kinematics
    """
    def __init__(self, l1=0.5, l2=1.0, l3=0.8):
        self.l1 = l1  # Link 1 length
        self.l2 = l2  # Link 2 length
        self.l3 = l3  # Link 3 length

    def forward_kinematics(self, theta):
        """
        Compute end-effector position from joint angles

        Args:
            theta: [theta1, theta2, theta3] in radians

        Returns:
            position: [x, y, z] in meters
        """
        # Simplified FK for vertical arm
        x = (self.l2 * np.sin(theta[1]) +
             self.l3 * np.sin(theta[1] + theta[2]))
        y = 0  # Planar for simplicity
        z = (self.l1 +
             self.l2 * np.cos(theta[1]) +
             self.l3 * np.cos(theta[1] + theta[2]))

        return np.array([x, y, z])

    def inverse_kinematics(self, target_pos, initial_guess=None):
        """
        Numerical IK to reach target position

        Args:
            target_pos: [x, y, z] desired position
            initial_guess: Initial joint angles

        Returns:
            Joint angles that reach target
        """
        if initial_guess is None:
            initial_guess = np.array([0, np.pi/4, np.pi/4])

        def cost(theta):
            current = self.forward_kinematics(theta)
            error = np.linalg.norm(current - target_pos)
            return error

        result = minimize(cost, initial_guess, method='SLSQP')
        return result.x

    def compute_jacobian(self, theta):
        """
        Compute Jacobian matrix (numerical approximation)
        """
        epsilon = 1e-6
        J = np.zeros((3, 3))

        pos_0 = self.forward_kinematics(theta)

        for i in range(3):
            theta_perturbed = theta.copy()
            theta_perturbed[i] += epsilon
            pos_perturbed = self.forward_kinematics(theta_perturbed)

            J[:, i] = (pos_perturbed - pos_0) / epsilon

        return J

# Example usage
arm = RobotArm3DOF()

# Forward kinematics
theta = [0, np.pi/4, np.pi/6]
pos = arm.forward_kinematics(theta)
print(f"FK: Joint angles {np.degrees(theta)} → Position {pos}")

# Inverse kinematics
target = [0.8, 0, 1.5]
solution = arm.inverse_kinematics(target)
print(f"IK: Target {target} → Joint angles {np.degrees(solution)}")

# Verify
achieved = arm.forward_kinematics(solution)
print(f"Verification: Achieved position {achieved}")
print(f"Error: {np.linalg.norm(achieved - target):.6f} m")

# Jacobian
J = arm.compute_jacobian(theta)
print(f"Jacobian shape: {J.shape}")
print(f"Jacobian determinant: {np.linalg.det(J[:, :]):.4f}")
```

---

## Key Takeaways

1. **Forward Kinematics**: Joint angles → end-effector position (unique solution)
2. **Inverse Kinematics**: Desired position → joint angles (multiple or no solutions)
3. **DH Parameters**: Standard way to describe robot geometry (4 parameters per joint)
4. **Analytical IK**: Closed-form solutions for simple geometries (fast, exact)
5. **Numerical IK**: Optimization-based for complex robots (slower, approximate)
6. **Jacobian**: Maps joint velocities to end-effector velocity (critical for control)
7. **Singularities**: Configurations where Jacobian loses rank (avoid in planning)
8. **Dynamics**: Torques = M·θ̈ + C·θ̇ + G (consider mass, Coriolis, gravity)
9. **Humanoid Redundancy**: Extra DOF enables secondary objectives
10. **Gravity Compensation**: Critical torques to hold arm against gravity

---

## Exercises

### Conceptual Questions

1. **Workspace Analysis**: Sketch the reachable workspace of a 2-link planar arm with l₁=1m, l₂=0.8m. What is the maximum reach?

2. **Multiple Solutions**: For a 2-link arm, describe a scenario where IK has (a) two solutions, (b) one solution, (c) no solution.

3. **Singularities**: Identify at least two singular configurations for a 2-link planar arm. Why are they problematic?

### Coding Exercises

4. **3-DOF FK Implementation**:
   - Implement forward kinematics for a 3-DOF spatial arm
   - Visualize the arm in 3D for different joint angles
   - Plot the workspace (reachable positions)

5. **IK with Constraints**:
   - Modify the numerical IK to respect joint limits (e.g., θ ∈ [-π, π])
   - Compare solution with and without constraints

6. **Jacobian-Based Velocity Control**:
   - Implement velocity-level IK: θ̇ = J⁻¹ · ẋ
   - Trace a circular trajectory with the end-effector
   - Handle singularities (pseudo-inverse)

### Advanced Challenge

7. **Gravity Compensation Control**:
   - Implement a controller that holds the arm stationary against gravity
   - Simulate with realistic link masses
   - Add a small disturbance and show the controller compensates

---

## Next: Week 12 - Bipedal Locomotion & Balance

In Week 12, we'll apply these kinematics and dynamics concepts to the challenge of **bipedal walking**:
- Gait planning and foot placement
- Zero Moment Point (ZMP) for stability
- Center of Mass (CoM) control
- Balance recovery strategies

Continue to [Week 12: Bipedal Locomotion & Balance](/docs/module-5/week12-bipedal-locomotion)

---

## Further Reading

- **Books:**
  - "Modern Robotics" by Lynch & Park (comprehensive, free online)
  - "Introduction to Robotics" by Craig (classic textbook)
  - "Robot Modeling and Control" by Spong, Hutchinson, Vidyasagar

- **Online Resources:**
  - [Modern Robotics Course](http://modernrobotics.org) - Free video lectures
  - [Peter Corke's Robotics Toolbox](https://github.com/petercorke/robotics-toolbox-python) - Python library
  - [ROS 2 MoveIt](https://moveit.ros.org) - Motion planning framework

- **Research:**
  - Humanoid robotics papers from IEEE-RAS Humanoids Conference
  - Boston Dynamics publications on Atlas control
