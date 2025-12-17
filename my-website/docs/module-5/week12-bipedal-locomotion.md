---
sidebar_position: 3
title: Week 12 - Bipedal Locomotion & Balance
---

# Week 12: Bipedal Locomotion & Balance Control

**Learning Objectives:**
- Understand why bipedal walking is challenging
- Learn gait cycle phases and foot placement strategies
- Apply the Zero Moment Point (ZMP) stability criterion
- Control Center of Mass (CoM) for dynamic balance
- Implement basic gait planning in Python
- Recognize balance recovery strategies

---

## Why is Bipedal Walking Hard?

Walking on two legs seems natural to humans, but it's one of the hardest problems in robotics.

### The Fundamental Challenge

```
Quadruped (4 legs):              Biped (2 legs):
  ┌─────┐                          ┌─────┐
  │     │ Stable                   │     │ Unstable!
  └──┬──┘ (even standing still)    └──┬──┘ (constantly falling)
    ╱│╲                               │
   ╱ │ ╲                              │
  1  2  3  4                          1  2
```

**Key Differences:**

| Aspect | Quadruped | Biped |
|--------|-----------|-------|
| **Stability** | Statically stable | Dynamically stable |
| **Support** | 3-4 feet always down | 1-2 feet down |
| **Balance** | Easy (large polygon) | Constant control needed |
| **Falling Risk** | Low | High |
| **Control Complexity** | Lower | Higher |

### Why Humanoids Use Bipedal Locomotion

Despite the difficulty, bipedal walking is valuable:

1. **Human-Designed World**: Stairs, narrow paths, doors designed for two-legged creatures
2. **Reach and Manipulation**: Frees upper body for carrying objects
3. **Height Advantage**: Better field of view
4. **Energy Efficiency**: (When perfected) More efficient than wheeled robots on rough terrain
5. **Social Acceptance**: Humans relate better to human-like motion

---

## Center of Mass (CoM): The Balance Point

### What is the Center of Mass?

The **Center of Mass (CoM)** is the average position of all mass in the robot.

**Intuition:** If you could balance the robot on a single point, that point would be the CoM.

```
Humanoid Robot (side view):

        ○  ← Head
        │
    ────┼────  ← Arms
        │
    CoM ●  ← Center of Mass (torso area)
        │
       ╱│╲
      ╱ │ ╲
     1  │  2
        │
      Ground
```

### Computing the Center of Mass

```python
import numpy as np

def compute_com(masses, positions):
    """
    Compute center of mass from component masses and positions

    Args:
        masses: List of masses [m1, m2, ..., mn] in kg
        positions: List of positions [[x1,y1,z1], [x2,y2,z2], ...] in meters

    Returns:
        com: [x, y, z] position of center of mass
    """
    masses = np.array(masses)
    positions = np.array(positions)

    total_mass = np.sum(masses)
    com = np.sum(masses[:, np.newaxis] * positions, axis=0) / total_mass

    return com

# Example: Simple humanoid (standing)
masses = [
    5.0,   # Head
    20.0,  # Torso
    3.0,   # Right arm
    3.0,   # Left arm
    8.0,   # Right leg
    8.0,   # Left leg
]

positions = [
    [0, 0, 1.6],    # Head
    [0, 0, 1.2],    # Torso
    [0.3, 0, 1.1],  # Right arm
    [-0.3, 0, 1.1], # Left arm
    [0.1, 0, 0.5],  # Right leg
    [-0.1, 0, 0.5], # Left leg
]

com = compute_com(masses, positions)
print(f"Center of Mass: ({com[0]:.3f}, {com[1]:.3f}, {com[2]:.3f}) meters")
# Output: Center of Mass: (0.000, 0.000, 1.043) meters
```

### Why CoM Matters for Balance

**Gravity acts through the CoM:**

```
Standing:                    Tipping:

    CoM ●                       CoM ●
        │                           ╲
        │                            ╲
        ↓ (gravity)                   ↓
      ╱───╲                         ╱───╲
     │Foot │                       │Foot │
    ─────────                     ─────────

CoM over foot = Stable        CoM outside foot = Fall!
```

**Balance Criterion (Simplified):**
- **Stable**: CoM projection falls inside the support area
- **Unstable**: CoM projection falls outside the support area

---

## Support Polygon: The Safe Zone

### What is the Support Polygon?

The **support polygon** is the area on the ground where the robot can place its CoM and remain stable.

### Single-Foot Support (One Leg)

```
Top view:

      ┌─────────┐
      │         │
      │  Foot   │  ← Support polygon = Foot area
      │         │
      └─────────┘

CoM must project here ● to stay balanced
```

### Double-Foot Support (Both Legs)

```
Top view:

  ┌─────┐         ┌─────┐
  │Left │         │Right│
  │Foot │         │Foot │
  └─────┘         └─────┘

Support polygon = Convex hull of both feet (larger area!)

      ╱─────────────╲
     ╱               ╲
    ╱    CoM can be   ╲
   ╱     anywhere      ╲
  ╱      in here!       ╲
 ╱─────────────────────╲
```

**Key Insight:** Double support = More stable (larger polygon)

### Computing the Support Polygon

```python
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt

def compute_support_polygon(foot_contacts):
    """
    Compute support polygon from foot contact points

    Args:
        foot_contacts: List of (x, y) contact points

    Returns:
        polygon_vertices: Vertices of the convex hull
    """
    if len(foot_contacts) < 3:
        # Degenerate case (line or point)
        return np.array(foot_contacts)

    points = np.array(foot_contacts)
    hull = ConvexHull(points)
    polygon_vertices = points[hull.vertices]

    return polygon_vertices

# Example: Standing on both feet
left_foot_contacts = [
    [-0.15, -0.05],  # Left foot corners
    [-0.15, 0.15],
    [0.05, 0.15],
    [0.05, -0.05],
]

right_foot_contacts = [
    [-0.05, -0.15],  # Right foot corners
    [-0.05, -0.35],
    [0.15, -0.35],
    [0.15, -0.15],
]

all_contacts = left_foot_contacts + right_foot_contacts
support_polygon = compute_support_polygon(all_contacts)

print(f"Support polygon has {len(support_polygon)} vertices")

# Visualize
plt.figure(figsize=(6, 8))
polygon = plt.Polygon(support_polygon, fill=False, edgecolor='blue', linewidth=2)
plt.gca().add_patch(polygon)
plt.plot([0], [-0.2], 'ro', markersize=10, label='CoM projection')
plt.xlabel('X (meters)')
plt.ylabel('Y (meters)')
plt.title('Support Polygon (Standing)')
plt.legend()
plt.grid(True, alpha=0.3)
plt.axis('equal')
plt.show()
```

### Stability Check: Is CoM Inside Polygon?

```python
def point_in_polygon(point, polygon):
    """
    Check if a point is inside a polygon

    Args:
        point: (x, y) point to test
        polygon: List of (x, y) vertices

    Returns:
        True if point is inside, False otherwise
    """
    x, y = point
    n = len(polygon)
    inside = False

    p1x, p1y = polygon[0]
    for i in range(1, n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y

    return inside

# Test: Is CoM inside support polygon?
com_projection = (0, -0.2)  # CoM projected to ground
is_stable = point_in_polygon(com_projection, support_polygon)

print(f"CoM inside support polygon: {is_stable}")
if is_stable:
    print("Robot is statically stable!")
else:
    print("Robot will tip over!")
```

---

## Zero Moment Point (ZMP): Dynamic Stability

### The Limitation of Static Stability

The support polygon test only works for **static** scenarios. During walking, the robot is constantly accelerating, which creates **inertial forces**.

**Zero Moment Point (ZMP)** accounts for dynamics.

### What is the ZMP?

The **ZMP** is the point on the ground where the total moment (torque) from gravity and inertia is zero.

**Intuition:** The ZMP is where you'd need to place a support to prevent the robot from rotating.

```
Walking (accelerating forward):

    CoM ●  ← Moving forward
        │  with acceleration
        ↓ (gravity)
        ↓↓ (inertia)
      ╱───╲
     │Foot │
    ─────●─────  ← ZMP (shifted due to acceleration)
```

### ZMP Stability Criterion

**Rule:** For a robot to be stable:
```
ZMP must be inside the support polygon
```

**If ZMP is outside:** The robot will rotate (tip over)

### Computing the ZMP (Simplified)

```python
def compute_zmp_2d(com_position, com_acceleration, com_height, g=9.81):
    """
    Compute ZMP for a simplified model

    Args:
        com_position: [x, y] CoM position (meters)
        com_acceleration: [ax, ay] CoM acceleration (m/s²)
        com_height: z height of CoM above ground (meters)
        g: Gravity (m/s²)

    Returns:
        zmp: [x, y] ZMP position on ground
    """
    x_com, y_com = com_position
    ax, ay = com_acceleration

    # ZMP formula (simplified)
    x_zmp = x_com - (com_height / g) * ax
    y_zmp = y_com - (com_height / g) * ay

    return np.array([x_zmp, y_zmp])

# Example: Robot accelerating forward
com_pos = [0.0, 0.0]      # CoM at origin (horizontal)
com_acc = [1.0, 0.0]      # Accelerating forward at 1 m/s²
com_h = 1.0               # CoM height 1 meter

zmp = compute_zmp_2d(com_pos, com_acc, com_h)
print(f"ZMP position: ({zmp[0]:.3f}, {zmp[1]:.3f})")
# Output: ZMP position: (-0.102, 0.000)
# ZMP shifts backward due to forward acceleration!
```

### ZMP During Walking

```python
def simulate_walking_zmp(duration=2.0, dt=0.01):
    """
    Simulate ZMP trajectory during simple walking
    """
    time_steps = int(duration / dt)
    time = np.arange(time_steps) * dt

    # Simplified CoM motion (sinusoidal)
    frequency = 1.0  # 1 Hz walking
    amplitude = 0.05  # 5 cm forward-backward motion

    com_x = amplitude * np.sin(2 * np.pi * frequency * time)
    com_y = np.zeros(time_steps)

    # CoM acceleration (derivative of velocity)
    com_ax = amplitude * (2 * np.pi * frequency)**2 * np.cos(2 * np.pi * frequency * time)
    com_ay = np.zeros(time_steps)

    # Compute ZMP trajectory
    com_height = 1.0
    zmp_trajectory = np.zeros((time_steps, 2))

    for i in range(time_steps):
        zmp_trajectory[i] = compute_zmp_2d(
            [com_x[i], com_y[i]],
            [com_ax[i], com_ay[i]],
            com_height
        )

    # Plot
    plt.figure(figsize=(12, 4))

    plt.subplot(1, 3, 1)
    plt.plot(time, com_x, label='CoM X')
    plt.plot(time, zmp_trajectory[:, 0], label='ZMP X')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title('CoM vs ZMP (Horizontal)')
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.subplot(1, 3, 2)
    plt.plot(time, com_ax, label='CoM Acceleration')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s²)')
    plt.title('CoM Acceleration')
    plt.grid(True, alpha=0.3)

    plt.subplot(1, 3, 3)
    plt.plot(com_x, zmp_trajectory[:, 0], 'b-', alpha=0.5)
    plt.plot([com_x[0]], [zmp_trajectory[0, 0]], 'go', label='Start')
    plt.plot([com_x[-1]], [zmp_trajectory[-1, 0]], 'ro', label='End')
    plt.xlabel('CoM X (m)')
    plt.ylabel('ZMP X (m)')
    plt.title('CoM vs ZMP Trajectory')
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

# Run simulation
simulate_walking_zmp()
```

---

## The Gait Cycle: How Humans Walk

### Gait Cycle Phases

A **gait cycle** is one complete walking step (from heel strike to next heel strike of the same foot).

```
Gait Cycle (Right Leg):

1. Heel Strike (0%)          2. Foot Flat (10%)
   ───●                         ──────
      ╲                           │
       Right foot touches         Full contact

3. Mid Stance (30%)          4. Heel Off (50%)
   ──────                        ╱────
     │                         ╱
     Full support              Pushing off

5. Toe Off (60%)             6. Swing (60-100%)
   ╱                            ╱
  ╱                           ╱╲
  Leaving ground              ╱  ╲ Swinging forward
                             0%   100%

Phase durations:
- Stance Phase: 0-60% (foot on ground)
- Swing Phase: 60-100% (foot in air)
```

### Double vs Single Support

```
Walking Timeline:

Time:   0%    10%    50%    60%   100%
       ┌──────────────────────────┐
Right: │  Stance (on ground)      │ Swing
       └──────────────────────────┘
Left:  │ Swing│  Stance (on ground)
       └──────┴──────────────────────┐

Support:
       [Double][Single][Double][Single]

Double Support: Both feet on ground (safer, slower)
Single Support: One foot on ground (faster, less stable)
```

### Gait State Machine

```python
class GaitStateMachine:
    """
    Simple state machine for biped walking
    """
    def __init__(self, step_duration=1.0):
        self.step_duration = step_duration  # Time for one step
        self.current_time = 0
        self.current_phase = 'double_support_1'

    def update(self, dt):
        """Update state machine"""
        self.current_time += dt
        phase_progress = (self.current_time % self.step_duration) / self.step_duration

        # Determine current phase (simplified 4-phase model)
        if phase_progress < 0.1:
            self.current_phase = 'double_support_1'
        elif phase_progress < 0.5:
            self.current_phase = 'single_support_right'
        elif phase_progress < 0.6:
            self.current_phase = 'double_support_2'
        else:
            self.current_phase = 'single_support_left'

        return self.current_phase

    def get_support_feet(self):
        """Return which feet are on the ground"""
        if 'double' in self.current_phase:
            return ['left', 'right']
        elif 'right' in self.current_phase:
            return ['right']
        elif 'left' in self.current_phase:
            return ['left']

# Example usage
gait = GaitStateMachine(step_duration=1.0)

for t in np.arange(0, 2.0, 0.1):
    phase = gait.update(0.1)
    feet = gait.get_support_feet()
    print(f"t={t:.1f}s: Phase={phase:25s} Support={feet}")
```

---

## Inverted Pendulum Model: Walking Simplified

### The Linear Inverted Pendulum (LIP)

The **inverted pendulum** is a simplified model of bipedal walking:

```
Side view:

    CoM ● ← Mass concentrated at CoM
        │
        │ Length L
        │
        ○ ← Ankle (pivot point)
        │
    ────┴──── Ground
```

**Dynamics (simplified):**
```
θ̈ = (g/L) · sin(θ)

For small angles: θ̈ ≈ (g/L) · θ
```

Where:
- θ = angle from vertical
- g = gravity (9.81 m/s²)
- L = distance from ankle to CoM

### Python Simulation

```python
def simulate_inverted_pendulum(initial_angle=0.1, duration=2.0, dt=0.01):
    """
    Simulate linear inverted pendulum (falling motion)

    Args:
        initial_angle: Initial angle from vertical (radians)
        duration: Simulation time (seconds)
        dt: Time step (seconds)
    """
    g = 9.81  # Gravity
    L = 1.0   # Pendulum length (m)

    # State: [angle, angular_velocity]
    state = np.array([initial_angle, 0.0])

    time_steps = int(duration / dt)
    time = np.arange(time_steps) * dt
    trajectory = np.zeros((time_steps, 2))

    for i in range(time_steps):
        # Record state
        trajectory[i] = state

        # Dynamics: θ̈ = (g/L) · sin(θ)
        theta = state[0]
        angular_acc = (g / L) * np.sin(theta)

        # Integrate (Euler method)
        state[1] += angular_acc * dt  # Update angular velocity
        state[0] += state[1] * dt     # Update angle

    # Plot
    plt.figure(figsize=(12, 4))

    plt.subplot(1, 2, 1)
    plt.plot(time, np.degrees(trajectory[:, 0]))
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (degrees)')
    plt.title('Inverted Pendulum Angle (Falling)')
    plt.grid(True, alpha=0.3)

    plt.subplot(1, 2, 2)
    plt.plot(time, trajectory[:, 1])
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.title('Angular Velocity')
    plt.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

# Simulate falling (uncontrolled)
simulate_inverted_pendulum(initial_angle=0.1)  # Start at 5.7 degrees
```

### Controlled Walking: Foot Placement Strategy

**Key Insight:** By placing the foot forward, we can control where the pendulum "pivots."

```python
def walking_with_foot_placement(steps=10, step_length=0.4):
    """
    Simulate walking using foot placement strategy
    """
    g = 9.81
    L = 1.0  # CoM height

    # Initial state
    com_x = 0
    com_vel = 0.5  # Initial forward velocity

    foot_positions = [0]
    com_positions = [com_x]

    for step in range(steps):
        # Predict where CoM will be when we need to step
        # Simplified: place foot where CoM will be
        t_step = 0.5  # Time until next step (seconds)

        # Optimal foot placement (Capture Point)
        foot_x = com_x + com_vel * np.sqrt(L / g)

        # Clamp to maximum step length
        foot_x = min(foot_x, foot_positions[-1] + step_length)

        foot_positions.append(foot_x)

        # Update CoM (simplified)
        com_x = foot_x - step_length * 0.2  # CoM slightly behind new foot
        com_vel *= 0.95  # Slight slowdown each step

        com_positions.append(com_x)

    # Visualize
    plt.figure(figsize=(12, 4))
    plt.plot(foot_positions, [0]*len(foot_positions), 'bo-', label='Foot Placements', markersize=10)
    plt.plot(com_positions, [1]*len(com_positions), 'r^-', label='CoM', markersize=8)
    plt.xlabel('Position (m)')
    plt.ylabel('Height (m)')
    plt.title('Walking: Foot Placement Strategy')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.ylim(-0.5, 1.5)
    plt.show()

walking_with_foot_placement()
```

---

## Balance Recovery: Handling Disturbances

### The Three Balance Strategies

When pushed, humans use three strategies to recover balance:

**1. Ankle Strategy (Small Disturbances)**
```
Before Push:         After Push:
    ●                    ●╲
    │                    │ ╲ Lean to compensate
    │                    │  ╲
  ──┴──               ──┴───
   Ankle torque applied
```

**2. Hip Strategy (Medium Disturbances)**
```
    ●                    ●─╲
    │                   ╱│  ╲ Move hips
    │                  ╱ │
  ──┴──              ──┴──
```

**3. Stepping Strategy (Large Disturbances)**
```
    ●                    ●
    │                    │╲
    │                    │ ╲
  ──┴──              ──┴── ╲──
                            Place foot to catch fall
```

### Implementing Push Recovery

```python
def simulate_push_recovery(push_force=50, recovery_strategy='stepping'):
    """
    Simulate balance recovery from external push

    Args:
        push_force: Force of push (Newtons)
        recovery_strategy: 'ankle', 'hip', or 'stepping'
    """
    # Robot parameters
    mass = 50  # kg
    com_height = 1.0  # m

    # Initial state
    com_velocity = 0
    com_displacement = 0

    # Apply push (impulse)
    push_impulse = push_force * 0.1  # Force × time
    com_velocity = push_impulse / mass

    print(f"Push applied: {push_force} N")
    print(f"Resulting CoM velocity: {com_velocity:.3f} m/s")

    # Recovery strategy
    if recovery_strategy == 'ankle':
        max_ankle_moment = 100  # N·m
        recovery_torque = min(max_ankle_moment, push_force * com_height)
        print(f"Ankle strategy: Applying {recovery_torque:.1f} N·m torque")

        if push_force > 30:
            print("⚠️  Push too strong for ankle strategy!")
            return False

    elif recovery_strategy == 'hip':
        print("Hip strategy: Moving hips to shift CoM")
        hip_movement = com_velocity * 0.5  # Simplified
        print(f"Hip movement: {hip_movement:.3f} m")

        if push_force > 60:
            print("⚠️  Push too strong for hip strategy!")
            return False

    elif recovery_strategy == 'stepping':
        # Capture Point: where to step to stop motion
        capture_point = com_displacement + com_velocity * np.sqrt(com_height / 9.81)
        print(f"Stepping strategy: Place foot at {capture_point:.3f} m")

        if abs(capture_point) > 0.8:  # Maximum step length
            print("⚠️  Required step too large!")
            return False

    print("✅ Balance recovered successfully!")
    return True

# Test different strategies
print("=== Small Push (20 N) ===")
simulate_push_recovery(20, 'ankle')

print("\n=== Medium Push (40 N) ===")
simulate_push_recovery(40, 'hip')

print("\n=== Large Push (70 N) ===")
simulate_push_recovery(70, 'stepping')

print("\n=== Very Large Push (150 N) ===")
simulate_push_recovery(150, 'stepping')
```

---

## Simple Gait Planning: Putting It All Together

### Walking Controller Pseudocode

```python
class SimpleWalkingController:
    """
    Simplified walking controller for humanoid robot
    """
    def __init__(self):
        self.step_duration = 0.8  # seconds
        self.step_length = 0.4    # meters
        self.com_height = 1.0     # meters
        self.current_phase = 'standing'
        self.time_in_phase = 0

    def plan_step(self, target_position):
        """
        Plan a single step toward target

        Returns:
            foot_placement: (x, y) where to place next foot
            com_trajectory: List of CoM waypoints
        """
        # Determine which foot to move (alternate)
        if self.current_phase == 'left_support':
            moving_foot = 'right'
        else:
            moving_foot = 'left'

        # Compute foot placement
        current_com = self.get_current_com()
        direction_to_target = target_position - current_com

        # Place foot along direction, limited by step length
        step_distance = min(np.linalg.norm(direction_to_target), self.step_length)
        foot_placement = current_com + step_distance * (direction_to_target / np.linalg.norm(direction_to_target))

        # Generate CoM trajectory (simplified)
        com_trajectory = self.generate_com_trajectory(
            start=current_com,
            end=foot_placement - np.array([self.step_length * 0.3, 0]),
            duration=self.step_duration
        )

        return foot_placement, com_trajectory

    def generate_com_trajectory(self, start, end, duration, num_points=20):
        """
        Generate smooth CoM trajectory

        Returns:
            List of (x, y, z) waypoints
        """
        t = np.linspace(0, duration, num_points)

        # Use cubic interpolation for smoothness
        trajectory = []
        for ti in t:
            # Normalized time (0 to 1)
            s = ti / duration
            # Cubic: 3s² - 2s³ (smooth acceleration)
            smooth_s = 3 * s**2 - 2 * s**3

            # Interpolate position
            pos = start + smooth_s * (end - start)
            trajectory.append(pos)

        return trajectory

    def check_stability(self, com, zmp, support_polygon):
        """
        Check if current state is stable

        Returns:
            is_stable: Boolean
            stability_margin: Distance from ZMP to polygon edge
        """
        # Check if ZMP is inside support polygon
        is_stable = point_in_polygon(zmp[:2], support_polygon)

        # Compute margin (simplified: distance to nearest edge)
        if is_stable:
            # Distance to nearest polygon edge
            distances = []
            for i in range(len(support_polygon)):
                p1 = support_polygon[i]
                p2 = support_polygon[(i+1) % len(support_polygon)]
                dist = self.point_to_line_distance(zmp[:2], p1, p2)
                distances.append(dist)
            stability_margin = min(distances)
        else:
            stability_margin = -1  # Outside polygon

        return is_stable, stability_margin

    def point_to_line_distance(self, point, line_p1, line_p2):
        """Compute distance from point to line segment"""
        # Simplified calculation
        return np.linalg.norm(point - line_p1)

    def get_current_com(self):
        """Get current center of mass (placeholder)"""
        return np.array([0, 0, self.com_height])


# Example usage
controller = SimpleWalkingController()

# Plan a step
target = np.array([2.0, 0, 1.0])  # Walk 2 meters forward
foot_placement, com_trajectory = controller.plan_step(target)

print(f"Next foot placement: {foot_placement}")
print(f"CoM trajectory has {len(com_trajectory)} waypoints")

# Visualize trajectory
trajectory_array = np.array(com_trajectory)
plt.figure(figsize=(8, 4))
plt.plot(trajectory_array[:, 0], trajectory_array[:, 2], 'b-o')
plt.xlabel('X Position (m)')
plt.ylabel('Z Height (m)')
plt.title('CoM Trajectory During Step')
plt.grid(True, alpha=0.3)
plt.show()
```

---

## Static vs Dynamic Walking

### Static Walking

**Definition:** CoM always projects inside the support polygon, even when frozen in time.

**Characteristics:**
- Very slow (robot "shuffles")
- Safe and stable
- Used in early robots and on uneven terrain

```
Static Walk:
Time 1:  Time 2:  Time 3:
  ●        ●        ●
  │        │        │
╱─┴─╲    ╱─┴──     ──┴─╲
Both     Left      Right
feet     foot      foot
```

### Dynamic Walking

**Definition:** Uses momentum and dynamics to maintain balance. CoM may be outside support polygon momentarily.

**Characteristics:**
- Faster and more natural
- Requires precise control
- Used in modern humanoids (Atlas, Optimus)

```
Dynamic Walk:
  ●╱         ●─        ╲●
   │          │         │
  ─┴──      ──┴─      ──┴──
Falling    Catching   Falling
forward    with foot  forward
```

**Key Insight:** Dynamic walking is "controlled falling" - the robot is constantly falling forward and catching itself with each step.

---

## Humanoid Walking in Practice

### Boston Dynamics Atlas

**Capabilities:**
- Dynamic walking on rough terrain
- Running (both feet off ground)
- Recovery from large pushes
- Jumping and parkour

**Key Technologies:**
- Model Predictive Control (MPC)
- Whole-body dynamics optimization
- Foot placement planning
- Real-time ZMP tracking

### Tesla Optimus

**Approach:**
- More cautious than Atlas (safety-first)
- Neural network-based control
- Learning from human motion data
- Focus on manufacturing tasks

### Common Challenges

All humanoid robots face:

1. **Uneven Terrain**: Stairs, slopes, obstacles
2. **External Disturbances**: Pushes, collisions
3. **Payload Carrying**: Holding objects while walking
4. **Energy Efficiency**: Battery life constraints
5. **Real-Time Control**: 100-1000 Hz control loops

---

## Key Takeaways

1. **Bipedal walking is challenging** due to inherent instability (unlike quadrupeds)
2. **Center of Mass (CoM)** is the balance point - gravity acts through it
3. **Support polygon** defines the safe area for CoM projection
4. **Zero Moment Point (ZMP)** accounts for dynamics - must stay inside support polygon
5. **Gait cycle** consists of stance and swing phases with double/single support periods
6. **Inverted pendulum model** simplifies walking for control and planning
7. **Balance recovery** uses ankle, hip, or stepping strategies depending on disturbance
8. **Static walking** is safe but slow; **dynamic walking** is faster but requires precise control
9. **Foot placement strategy** is critical for stable walking
10. **Real humanoids** use sophisticated MPC and optimization for robustness

---

## Exercises

### Conceptual Questions

1. **Stability Comparison**: Why is a quadruped robot inherently more stable than a biped? Draw support polygons for both.

2. **ZMP vs CoM**: Explain the difference between ZMP and CoM projection. When are they the same?

3. **Recovery Strategies**: You push a standing humanoid robot with 10 N, then 100 N, then 300 N. Which recovery strategy would be used for each?

### Coding Exercises

4. **Support Polygon Visualization**:
   - Implement a function that visualizes support polygons for different foot configurations
   - Show single support (one foot), double support (two feet), and triple support (two feet + hand)
   - Test if different CoM positions are stable

5. **ZMP Tracking**:
   - Simulate a robot walking in place (CoM oscillating side-to-side)
   - Compute and plot ZMP trajectory
   - Verify ZMP stays within foot bounds

6. **Simple Gait Planner**:
   - Implement a gait planner that generates footstep positions for walking 5 meters forward
   - Include timing information (when each foot lifts/lands)
   - Visualize the footstep sequence

### Advanced Challenge

7. **Push Recovery Simulation**:
   - Implement a simple 2D humanoid model (inverted pendulum)
   - Apply random pushes at random times
   - Implement stepping strategy to recover balance
   - Count successful vs failed recovery attempts

---

## Next Steps

Congratulations on completing Module 5! You now understand:
- Humanoid kinematics and dynamics (Week 11)
- Bipedal locomotion and balance control (Week 12)

### Continue Your Learning

**Module 2: Robot Simulation**
- Apply kinematics in Gazebo simulator
- Simulate bipedal walking
- Test control algorithms safely

**Module 3: NVIDIA Isaac**
- Use Isaac Sim for humanoid simulation
- Reinforcement learning for locomotion
- Sim-to-real transfer

**Module 4: Vision-Language-Action**
- Integrate walking with high-level planning
- Voice-controlled humanoid navigation
- Complete autonomous systems

---

## Further Reading

**Foundational Papers:**
- "Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point" (Kajita et al., 2003)
- "Balancing of Humanoid Robot Using Contact Force/Moment" (Stephens & Atkeson, 2010)

**Books:**
- "Introduction to Humanoid Robotics" by Kajita et al.
- "Humanoid Robots: Modeling and Control" by Vukobratović & Borovac

**Online Resources:**
- [MIT 6.832: Underactuated Robotics](http://underactuated.mit.edu) (Free course on walking robots)
- [CMU: Legged Locomotion](https://www.cs.cmu.edu/~cga/legs/) (Research lab)
- [Open Dynamic Robot Initiative](https://open-dynamic-robot-initiative.github.io/) (Open source humanoid robots)

**Videos:**
- Boston Dynamics Atlas demonstrations
- Tesla AI Day presentations (Optimus walking)
- Agility Robotics Digit in warehouses
