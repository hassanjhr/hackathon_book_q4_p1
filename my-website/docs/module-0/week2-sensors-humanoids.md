---
sidebar_position: 2
title: Week 2 - Sensors & Humanoid Landscape
---

# Week 2: Sensors and the Humanoid Robotics Landscape

## Part 1: Sensor Systems - The Robot's Senses

Robots perceive the world through **sensors** - devices that convert physical phenomena (light, sound, force, motion) into electrical signals. Understanding sensor characteristics is critical for designing reliable robotic systems.

### Sensor Taxonomy

```
Sensors
├── Proprioceptive (Internal State)
│   ├── Encoders (joint angles)
│   ├── IMUs (orientation, acceleration)
│   └── Force/Torque (contact forces)
│
└── Exteroceptive (External World)
    ├── LIDAR (3D distance)
    ├── Cameras (vision)
    ├── Ultrasonic (proximity)
    └── Microphones (audio)
```

---

## 1. LIDAR (Light Detection and Ranging)

### Principle

LIDAR measures distance by emitting laser pulses and measuring the time-of-flight (TOF):

```
Distance = (Speed_of_Light × Time) / 2
```

### Types

**2D LIDAR (Planar):**
- Scans in a single plane (e.g., 360° horizontally)
- Common in mobile robots (e.g., vacuum cleaners)
- Output: Array of distances at different angles

**3D LIDAR (Spinning/Solid-State):**
- Multiple scanning planes or arrays
- Creates 3D point clouds
- Used in autonomous vehicles

### Python Example: Processing LIDAR Data

```python
import numpy as np
import matplotlib.pyplot as plt

class LIDARProcessor:
    def __init__(self, max_range=10.0):
        self.max_range = max_range

    def polar_to_cartesian(self, ranges, angles):
        """
        Convert LIDAR polar coordinates to Cartesian
        ranges: list of distances (meters)
        angles: list of angles (radians)
        """
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        return x, y

    def detect_obstacles(self, ranges, threshold=0.5):
        """
        Find obstacles closer than threshold
        """
        obstacles = []
        for i, distance in enumerate(ranges):
            if distance < threshold:
                angle = (i / len(ranges)) * 2 * np.pi
                obstacles.append((distance, angle))
        return obstacles

# Example usage
lidar = LIDARProcessor()

# Simulated LIDAR scan (360 points, 360 degrees)
angles = np.linspace(0, 2*np.pi, 360)
ranges = np.random.uniform(0.5, 10.0, 360)  # Random distances

# Convert to Cartesian
x, y = lidar.polar_to_cartesian(ranges, angles)

# Detect close obstacles
obstacles = lidar.detect_obstacles(ranges, threshold=1.0)
print(f"Detected {len(obstacles)} obstacles within 1m")
```

### Key Characteristics

| Property | Value |
|----------|-------|
| **Range** | 0.1m - 100m (depending on model) |
| **Accuracy** | ±1-3 cm |
| **Update Rate** | 5-40 Hz |
| **Field of View** | 360° (2D), full 3D (3D LIDAR) |
| **Limitations** | Transparent surfaces, mirrors, sunlight interference |

---

## 2. Cameras (Vision Sensors)

### Types

**RGB Cameras:**
- Standard color images
- Resolution: 640×480 to 4K+
- Used for object detection, tracking

**Depth Cameras (RGB-D):**
- Provide color + depth per pixel
- Examples: Intel RealSense, Microsoft Kinect
- Enable 3D perception

**Stereo Cameras:**
- Two cameras simulate human binocular vision
- Compute depth via triangulation

### Python Example: Camera-based Object Detection

```python
import cv2
import numpy as np

class CameraProcessor:
    def __init__(self):
        self.camera = cv2.VideoCapture(0)  # Open default camera

    def detect_red_objects(self, frame):
        """
        Simple color-based object detection
        """
        # Convert BGR to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define red color range
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])

        # Create mask
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Find contours
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        objects = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Filter small noise
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                center = (x + w//2, y + h//2)
                objects.append({
                    'center': center,
                    'area': area,
                    'bbox': (x, y, w, h)
                })

        return objects

    def estimate_distance(self, object_width_pixels, real_width_m=0.1):
        """
        Estimate distance using similar triangles
        Assumes known object width and camera focal length
        """
        focal_length = 500  # pixels (camera-dependent)
        distance = (real_width_m * focal_length) / object_width_pixels
        return distance

# Example usage
processor = CameraProcessor()

while True:
    ret, frame = processor.camera.read()
    if not ret:
        break

    # Detect red objects
    objects = processor.detect_red_objects(frame)

    for obj in objects:
        # Draw bounding box
        x, y, w, h = obj['bbox']
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # Estimate distance
        distance = processor.estimate_distance(w)
        cv2.putText(frame, f"{distance:.2f}m", (x, y-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow('Red Object Detection', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

processor.camera.release()
cv2.destroyAllWindows()
```

### Key Characteristics

| Property | RGB Camera | Depth Camera |
|----------|-----------|--------------|
| **Range** | Unlimited (limited by optics) | 0.5-10m |
| **Resolution** | 640×480 to 4K+ | 640×480 typical |
| **Frame Rate** | 30-120 fps | 30-60 fps |
| **Data Type** | Color images | Color + depth map |
| **Limitations** | Lighting dependent | IR interference, range limits |

---

## 3. IMU (Inertial Measurement Unit)

### Components

An IMU combines:
1. **Accelerometer** (3-axis): Measures linear acceleration
2. **Gyroscope** (3-axis): Measures angular velocity
3. **Magnetometer** (3-axis, optional): Measures magnetic field (compass)

### How It Works

```
IMU Output = Accelerometer + Gyroscope + (Magnetometer)
           ↓
    Sensor Fusion (e.g., Kalman Filter)
           ↓
    Orientation Estimate (Roll, Pitch, Yaw)
```

### Python Example: IMU Data Processing

```python
import numpy as np

class IMUProcessor:
    def __init__(self):
        # Sensor characteristics
        self.accel_noise = 0.01  # m/s²
        self.gyro_noise = 0.001  # rad/s

        # State
        self.orientation = np.array([0.0, 0.0, 0.0])  # Roll, Pitch, Yaw
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.position = np.array([0.0, 0.0, 0.0])

    def update(self, accel, gyro, dt):
        """
        Update state from IMU measurements
        accel: [ax, ay, az] in m/s²
        gyro: [gx, gy, gz] in rad/s
        dt: time step in seconds
        """
        # Integrate gyroscope for orientation
        self.orientation += gyro * dt

        # Remove gravity from accelerometer
        gravity = np.array([0, 0, -9.81])
        accel_world = self.rotate_to_world(accel) - gravity

        # Integrate acceleration for velocity and position
        self.velocity += accel_world * dt
        self.position += self.velocity * dt

        return self.orientation, self.velocity, self.position

    def rotate_to_world(self, accel):
        """
        Rotate accelerometer reading to world frame
        using current orientation estimate
        """
        # Simplified rotation (full implementation uses quaternions)
        roll, pitch, yaw = self.orientation

        # Rotation matrix (simplified)
        R = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw),  np.cos(yaw), 0],
            [0, 0, 1]
        ])

        return R @ accel

# Example usage
imu = IMUProcessor()

# Simulate IMU data (robot accelerating forward)
dt = 0.01  # 100 Hz
for t in np.arange(0, 5, dt):
    # Simulated sensor readings
    accel = np.array([1.0, 0, -9.81])  # 1 m/s² forward, gravity down
    gyro = np.array([0, 0, 0.1])       # Turning slightly

    # Update state
    orientation, velocity, position = imu.update(accel, gyro, dt)

    if t % 1.0 < dt:  # Print every second
        print(f"t={t:.1f}s: pos={position}, vel={velocity}")
```

### Key Characteristics

| Property | Value |
|----------|-------|
| **Update Rate** | 100-1000 Hz |
| **Accelerometer Range** | ±2g to ±16g |
| **Gyroscope Range** | ±250°/s to ±2000°/s |
| **Drift** | Accumulates over time (integration errors) |
| **Use Cases** | Orientation, balance control, motion tracking |

---

## 4. Force/Torque Sensors

### Principle

Force/torque sensors measure mechanical forces and moments applied to the sensor. Critical for:
- **Manipulation**: Detecting contact, grasping forces
- **Locomotion**: Ground reaction forces
- **Safety**: Collision detection

### Types

**Strain Gauge Based:**
- Measure deformation under load
- 6-axis (3 forces + 3 torques)

**Capacitive/Piezoelectric:**
- Measure force via electrical property changes

### Python Example: Grasp Force Control

```python
class ForceController:
    def __init__(self, target_force=10.0):
        self.target_force = target_force  # Newtons
        self.Kp = 0.5  # Proportional gain

    def control_grasp(self, measured_force):
        """
        Adjust gripper based on force feedback
        """
        error = self.target_force - measured_force

        # Proportional control
        gripper_command = self.Kp * error

        # Safety limits
        if measured_force > 50.0:
            print("WARNING: Force too high, releasing grip")
            return -5.0  # Open gripper

        return gripper_command

# Example usage
controller = ForceController(target_force=15.0)

# Simulate grasping an object
measured_forces = [0, 5, 10, 12, 15, 16, 15, 14, 15]

for force in measured_forces:
    command = controller.control_grasp(force)
    print(f"Force: {force}N → Command: {command:.2f}")
```

### Key Characteristics

| Property | Value |
|----------|-------|
| **Force Range** | 0-500N typical |
| **Torque Range** | 0-50 Nm typical |
| **Accuracy** | 0.5-2% of full scale |
| **Update Rate** | 100-1000 Hz |

---

## Part 2: The Humanoid Robotics Landscape

### What is a Humanoid Robot?

A **humanoid robot** is a robot with a human-like body structure:
- **Bipedal locomotion** (two legs)
- **Anthropomorphic arms** (shoulders, elbows, wrists)
- **Torso and head**
- **Hands with fingers** (manipulation)

### Why Humanoid Form?

1. **Human-designed world**: Buildings, tools, vehicles designed for humans
2. **Social interaction**: Humans relate better to human-like forms
3. **Versatility**: Can perform any human task (in theory)
4. **Testbed for AI**: Challenges perception, control, planning simultaneously

### Current State-of-the-Art

#### 1. Boston Dynamics Atlas

**Specifications:**
- Height: 1.5m
- Weight: 89kg
- Degrees of Freedom: 28
- Capabilities: Parkour, backflips, object manipulation

**Key Technologies:**
- Hydraulic actuation (high power-to-weight)
- Model-predictive control for dynamic motion
- Onboard perception (LIDAR, cameras)

#### 2. Tesla Optimus (Tesla Bot)

**Specifications:**
- Height: 1.73m (5'8")
- Weight: 57kg (125 lbs)
- Payload: 20kg (45 lbs)
- Walking Speed: 8 km/h

**Design Philosophy:**
- Mass production focus (automotive manufacturing)
- Electric actuation (simpler than hydraulics)
- AI-first approach (FSD computer, neural nets)

#### 3. Figure 01

**Specifications:**
- Height: 1.60m
- Weight: 60kg
- Battery Life: 5 hours
- Payload: 20kg

**Innovation:**
- All-electric design
- Integrated AI models (vision-language-action)
- Rapid iteration (startup approach)

#### 4. Agility Robotics Digit

**Specifications:**
- Height: 1.75m
- Weight: 65kg
- Focus: Warehouse logistics

**Unique Features:**
- Bird-like legs (digitigrade)
- Torso-less design (compact)
- Optimized for walking with loads

### Challenges in Humanoid Robotics

#### 1. Balance and Locomotion

Bipedal walking is inherently unstable:

```python
# Simplified balance control (Zero Moment Point - ZMP)
def compute_zmp(mass, com_position, com_acceleration, g=9.81):
    """
    ZMP must stay within support polygon for stability
    """
    zmp_x = com_position[0] - (com_position[2] / (g + com_acceleration[2])) * com_acceleration[0]
    zmp_y = com_position[1] - (com_position[2] / (g + com_acceleration[2])) * com_acceleration[1]

    return (zmp_x, zmp_y)

# Check if ZMP is within foot support
def is_stable(zmp, foot_polygon):
    """
    Stability check: ZMP inside support polygon?
    """
    # Point-in-polygon test
    return point_in_polygon(zmp, foot_polygon)
```

#### 2. Energy Efficiency

Humanoid robots consume significant power:

```
Energy Consumption Example (Tesla Optimus):
- Battery: 2.3 kWh
- Operating Time: 5 hours
- Average Power: 460W

Compare to humans: ~100W resting, ~400W walking
```

#### 3. Manipulation Dexterity

Human hands have 27 degrees of freedom. Robotic hands trade-off:
- **Complexity**: More DOF = harder control
- **Cost**: Sensors, actuators expensive
- **Robustness**: Simpler = more reliable

### Future Trends

1. **AI-Native Design**
   - Foundation models for manipulation
   - End-to-end learning (vision → action)
   - Sim-to-real transfer

2. **Mass Production**
   - Cost reduction (target: &lt;$50k per unit)
   - Modular designs
   - Standardization

3. **Applications**
   - Warehouse automation
   - Elder care
   - Household assistance
   - Hazardous environments

## Key Takeaways

1. **Sensors are the robot's interface to the world** - LIDAR, cameras, IMUs, force sensors each serve distinct roles
2. **Sensor fusion is critical** - Combining multiple sensors improves robustness
3. **Humanoid robotics is rapidly advancing** - From research (Atlas) to commercialization (Optimus, Figure 01)
4. **Balance and manipulation remain hard problems** - Physics and control are core challenges
5. **AI is becoming central** - Modern humanoids integrate vision-language-action models

## Exercises

1. **Sensor Selection**: Design a sensor suite for a warehouse robot. Which sensors would you choose and why?
2. **LIDAR Processing**: Write code to detect a doorway in LIDAR data (hint: look for gaps)
3. **Research**: Compare Atlas vs Optimus - what design trade-offs did each make?

## Next: Module 1 - ROS 2 Fundamentals

Now that you understand Physical AI principles and sensor systems, you're ready to learn the **software infrastructure** that ties everything together: ROS 2.

---

**Further Resources:**
- [Boston Dynamics Atlas](https://www.bostondynamics.com/atlas)
- [Tesla Optimus Updates](https://www.tesla.com/AI)
- [Figure AI](https://www.figure.ai/)
- IEEE Spectrum Robotics News
