---
sidebar_position: 3
title: Week 7 - Unity Visualization
---

# Week 7: Unity Visualization for Robotics

**Learning Objectives:**
- Understand when to use Unity vs Gazebo for robotics
- Set up ROS 2 ↔ Unity communication via ROS-TCP-Connector
- Import and control robot models in Unity
- Visualize sensor data (camera, LIDAR) in Unity
- Build interactive UIs for robot control

---

## Why Unity for Robotics?

Last week we used **Gazebo** for physics-based simulation. This week we explore **Unity**, a game engine that excels at **visualization, UI/UX, and human interaction**.

### Gazebo vs Unity: When to Use Each

| Feature | Gazebo | Unity |
|---------|--------|-------|
| **Primary Purpose** | Physics simulation | Visualization & interaction |
| **Graphics Quality** | Good (functional) | Excellent (photorealistic) |
| **Physics Accuracy** | High (ODE, Bullet) | Medium (PhysX, game-oriented) |
| **ROS Integration** | Native (`gazebo_ros`) | Via TCP bridge |
| **UI/UX Development** | Limited | Excellent (GUI, buttons, sliders) |
| **VR/AR Support** | None | Full support |
| **Animation** | Basic | Advanced (humanoid IK, mocap) |
| **Learning Curve** | Medium | Medium-High |
| **Use Case** | Algorithm development | Demos, teleoperation, HRI |

**Key Insight**: Use **Gazebo for testing algorithms** (path planning, control) and **Unity for visualization** (demos, user interfaces, remote operation).

### Unity's Strengths for Robotics

1. **Photorealistic Rendering**: Impress stakeholders, create training data
2. **UI Elements**: Buttons, sliders, graphs for teleoperation
3. **Animation**: Humanoid characters, inverse kinematics (IK)
4. **VR/AR**: Immersive robot control, maintenance training
5. **Cross-Platform**: Windows, Mac, Linux, WebGL (browser-based)

**Example Use Cases:**
- **Teleoperation Interface**: Control a warehouse robot with joystick + camera feed
- **Digital Twin**: Mirror real robot's state in real-time
- **Training Simulator**: VR environment for operator training
- **Human-Robot Interaction**: Social robots with facial expressions

---

## Setting Up Unity with ROS 2

### Prerequisites

**On Ubuntu (ROS 2 side):**
- ROS 2 Humble installed
- Python 3.10+

**On Windows/Mac/Linux (Unity side):**
- Unity Hub: Download from [unity.com](https://unity.com/download)
- Unity 2021.3 LTS or later (2022 LTS recommended)

### Step 1: Install Unity

1. Download **Unity Hub**
2. Install **Unity 2021.3 LTS** (or 2022 LTS)
   - Include modules: **Linux Build Support** (if deploying to Linux)
3. Create a new project:
   - Template: **3D (URP - Universal Render Pipeline)** for better graphics
   - Name: `RobotViz`

### Step 2: Install ROS-TCP-Connector (Unity Package)

**ROS-TCP-Connector** bridges Unity (C#) and ROS 2 (Python) via TCP sockets.

**In Unity:**
1. Open `Window → Package Manager`
2. Click `+` → `Add package from git URL`
3. Paste: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
4. Wait for installation

**Verify**: Check `Robotics → ROS Settings` in the menu bar.

### Step 3: Set Up ROS-TCP-Endpoint (ROS 2 side)

On your ROS 2 machine:

```bash
# Create workspace
mkdir -p ~/unity_ros_ws/src
cd ~/unity_ros_ws/src

# Clone ROS-TCP-Endpoint
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint

# Install dependencies
cd ~/unity_ros_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build

# Source
source install/setup.bash
```

**Run the endpoint:**
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

**Output:**
```
[INFO] [ros_tcp_endpoint]: Starting server on 0.0.0.0:10000
```

**What this does**: Opens a TCP server on port 10000 that Unity connects to. All ROS 2 messages are serialized and sent over TCP.

### Step 4: Configure Unity to Connect to ROS 2

**In Unity:**
1. `Robotics → ROS Settings`
2. Set **ROS IP Address**: Your ROS 2 machine's IP (e.g., `192.168.1.100` or `localhost` if same machine)
3. Set **ROS Port**: `10000` (default)
4. **Protocol**: `ROS 2`

**Test connection**: Unity should now communicate with ROS 2 when the endpoint is running.

---

## Example 1: Publishing Pose from ROS 2 to Unity

Let's send a robot's position from ROS 2 and visualize it in Unity.

### ROS 2 Publisher (Python)

```python
# pose_publisher.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import math

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')

        self.publisher = self.create_publisher(Pose, '/robot_pose', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz
        self.time = 0.0

    def timer_callback(self):
        """Publish a moving pose (circular motion)"""
        msg = Pose()

        # Circular motion
        radius = 2.0
        msg.position.x = radius * math.cos(self.time)
        msg.position.y = radius * math.sin(self.time)
        msg.position.z = 1.0

        # Orientation (facing forward)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(self.time / 2)
        msg.orientation.w = math.cos(self.time / 2)

        self.publisher.publish(msg)
        self.time += 0.05

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run it:**
```bash
# Terminal 1: ROS-TCP-Endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

# Terminal 2: Pose publisher
ros2 run my_unity_package pose_publisher
```

### Unity Subscriber (C#)

**Create a GameObject**:
1. In Unity, create an empty GameObject: `GameObject → Create Empty` (name it `Robot`)
2. Add a visual (e.g., `Cube`) as a child

**Attach script**:
1. Create C# script: `Assets → Create → C# Script` (name: `PoseSubscriber`)
2. Double-click to edit:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class PoseSubscriber : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        // Get ROS connection
        ros = ROSConnection.GetOrCreateInstance();

        // Register message type
        ros.RegisterRosService<PoseMsg>("/robot_pose");

        // Subscribe to topic
        ros.Subscribe<PoseMsg>("/robot_pose", PoseCallback);

        Debug.Log("Subscribed to /robot_pose");
    }

    void PoseCallback(PoseMsg poseMsg)
    {
        // Convert ROS coordinates to Unity coordinates
        // ROS: X=forward, Y=left, Z=up
        // Unity: X=right, Y=up, Z=forward
        Vector3 position = new Vector3(
            (float)poseMsg.position.x,   // ROS X → Unity X
            (float)poseMsg.position.z,   // ROS Z → Unity Y
            (float)poseMsg.position.y    // ROS Y → Unity Z
        );

        // Convert quaternion (same order, but swap Y/Z)
        Quaternion rotation = new Quaternion(
            (float)poseMsg.orientation.x,
            (float)poseMsg.orientation.z,
            (float)poseMsg.orientation.y,
            (float)poseMsg.orientation.w
        );

        // Apply to GameObject
        transform.position = position;
        transform.rotation = rotation;
    }
}
```

3. Attach script to `Robot` GameObject (drag onto Inspector)

**Run Unity**: Press Play. The cube should move in a circle as ROS 2 publishes poses.

**Coordinate System Note**: ROS uses **right-handed Z-up**, Unity uses **left-handed Y-up**. Always convert:
```
ROS → Unity:
  X → X
  Y → Z
  Z → Y
```

---

## Example 2: Controlling a Robot Arm with Joint States

Let's import a robot arm URDF and control it from ROS 2.

### Step 1: Import URDF into Unity

**Install URDF Importer:**
1. `Window → Package Manager`
2. `+ → Add package from git URL`
3. Paste: `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer`

**Import URDF:**
1. `Assets → Import Robot from URDF`
2. Select your `simple_arm.urdf` file (from Week 6)
3. Unity converts links to GameObjects, joints to Articulation Bodies

**Result**: Your robot arm appears in the scene with physics-based joints.

### Step 2: Subscribe to Joint States

**ROS 2 Publisher** (publishes joint positions):

```python
# joint_state_publisher.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')

        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.time = 0.0

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2']

        # Sinusoidal motion
        joint1_pos = 0.5 * math.sin(self.time)
        joint2_pos = 0.3 * math.cos(self.time + 1.0)

        msg.position = [joint1_pos, joint2_pos]
        msg.velocity = []  # Optional
        msg.effort = []    # Optional

        self.publisher.publish(msg)
        self.time += 0.05

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Unity Subscriber**:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System.Collections.Generic;

public class JointStateSubscriber : MonoBehaviour
{
    private ROSConnection ros;

    // Assign ArticulationBody components in Inspector
    public ArticulationBody[] joints;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>("/joint_states", JointStateCallback);
    }

    void JointStateCallback(JointStateMsg msg)
    {
        // Update each joint
        for (int i = 0; i < msg.position.Length && i < joints.Length; i++)
        {
            ArticulationBody joint = joints[i];
            ArticulationDrive drive = joint.xDrive;

            // Set target position (convert radians to degrees)
            drive.target = (float)msg.position[i] * Mathf.Rad2Deg;

            joint.xDrive = drive;
        }
    }
}
```

**Setup:**
1. Attach script to your robot's root GameObject
2. In Inspector, set **Joints** array size to 2
3. Drag `joint1` and `joint2` ArticulationBody components into the array

**Run**: Robot arm should move following ROS 2 joint commands.

---

## Example 3: Publishing from Unity to ROS 2

Let's send button clicks from Unity to ROS 2.

### Unity Publisher (C#)

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class ButtonPublisher : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>("/unity_button");
    }

    // Call this from a Unity UI Button's OnClick event
    public void OnButtonClick(string buttonName)
    {
        StringMsg msg = new StringMsg();
        msg.data = $"Button '{buttonName}' pressed";

        ros.Publish("/unity_button", msg);
        Debug.Log($"Published: {msg.data}");
    }
}
```

**Unity UI Setup:**
1. Create UI Button: `GameObject → UI → Button`
2. Attach `ButtonPublisher` script to a GameObject
3. In Button's Inspector:
   - `OnClick()` → Drag the GameObject
   - Select `ButtonPublisher.OnButtonClick`
   - Enter button name (e.g., "Start")

**ROS 2 Subscriber**:

```python
# button_listener.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ButtonListener(Node):
    def __init__(self):
        super().__init__('button_listener')

        self.subscription = self.create_subscription(
            String,
            '/unity_button',
            self.button_callback,
            10
        )

    def button_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

        # Trigger actions based on button
        if "Start" in msg.data:
            self.get_logger().info('Starting robot...')
        elif "Stop" in msg.data:
            self.get_logger().info('Stopping robot...')

def main(args=None):
    rclpy.init(args=args)
    node = ButtonListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Use case**: Teleoperation interface where operators click Unity buttons to control robots.

---

## Example 4: Visualizing Camera Feed in Unity

Display a ROS 2 camera image in Unity.

### ROS 2 Camera Publisher

(Use the camera from Week 6's Gazebo example, or simulate one)

```bash
ros2 launch my_robot_description robot_sim.launch.py
# Camera publishes to /robot/camera/image_raw
```

### Unity Camera Subscriber

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using UnityEngine.UI;

public class CameraSubscriber : MonoBehaviour
{
    private ROSConnection ros;

    // Assign a RawImage UI component in Inspector
    public RawImage displayImage;

    private Texture2D texture;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<ImageMsg>("/robot/camera/image_raw", ImageCallback);

        // Create texture
        texture = new Texture2D(2, 2);
    }

    void ImageCallback(ImageMsg msg)
    {
        // Resize texture if needed
        if (texture.width != (int)msg.width || texture.height != (int)msg.height)
        {
            texture.Resize((int)msg.width, (int)msg.height);
        }

        // Convert ROS image data to Unity texture
        // Assumes RGB8 encoding
        if (msg.encoding == "rgb8")
        {
            Color32[] pixels = new Color32[msg.width * msg.height];

            for (int i = 0; i < pixels.Length; i++)
            {
                int idx = i * 3;
                pixels[i] = new Color32(
                    msg.data[idx],     // R
                    msg.data[idx + 1], // G
                    msg.data[idx + 2], // B
                    255                // A
                );
            }

            texture.SetPixels32(pixels);
            texture.Apply();

            // Display
            displayImage.texture = texture;
        }
    }
}
```

**Unity UI Setup:**
1. Create UI: `GameObject → UI → Raw Image`
2. Attach script to a GameObject
3. Drag Raw Image into the `Display Image` field

**Result**: Live camera feed from ROS 2 displayed in Unity.

---

## Example 5: Humanoid Animation with Unity's Animator

Unity's **Humanoid Animator** is perfect for humanoid robots (like Figure 01, Optimus).

### Step 1: Import Humanoid Model

**Option 1**: Import a rigged humanoid model (FBX/GLTF) from:
- Unity Asset Store (free humanoid characters)
- Mixamo (free animated characters)

**Option 2**: Use URDF Importer for your humanoid robot

**Configure as Humanoid**:
1. Select the model in `Assets`
2. Inspector → `Rig` tab
3. Set **Animation Type** to `Humanoid`
4. Click `Configure` → Unity auto-maps bones (head, spine, arms, legs)

### Step 2: Control Animation from ROS 2

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class HumanoidAnimator : MonoBehaviour
{
    private ROSConnection ros;
    private Animator animator;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        animator = GetComponent<Animator>();

        ros.Subscribe<StringMsg>("/robot_animation", AnimationCallback);
    }

    void AnimationCallback(StringMsg msg)
    {
        string animationName = msg.data;

        // Trigger animations
        if (animationName == "walk")
        {
            animator.SetBool("isWalking", true);
        }
        else if (animationName == "wave")
        {
            animator.SetTrigger("Wave");
        }
        else if (animationName == "idle")
        {
            animator.SetBool("isWalking", false);
        }

        Debug.Log($"Animation: {animationName}");
    }
}
```

**ROS 2 Publisher**:

```python
# animation_commander.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AnimationCommander(Node):
    def __init__(self):
        super().__init__('animation_commander')

        self.publisher = self.create_publisher(String, '/robot_animation', 10)

    def send_animation(self, anim_name):
        msg = String()
        msg.data = anim_name
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent animation: {anim_name}')

def main(args=None):
    rclpy.init(args=args)
    node = AnimationCommander()

    # Example usage
    node.send_animation("walk")
    rclpy.spin_once(node, timeout_sec=0.5)

    node.send_animation("wave")
    rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Use case**: Humanoid robot plays contextual animations (waving when greeting, walking when navigating).

---

## Building a Teleoperation Dashboard

Let's create a complete teleoperation interface in Unity.

### Features:
- Camera feed display
- Joystick controls (WASD or gamepad)
- Status indicators (battery, connection)
- Emergency stop button

### Unity Scene Setup

1. **Canvas** (UI root):
   - Create: `GameObject → UI → Canvas`

2. **Camera Feed Panel**:
   - `Raw Image` (640×480) - displays camera
   - Label: "Robot Camera"

3. **Control Panel**:
   - Buttons: "Start", "Stop", "Emergency Stop"
   - Sliders: "Speed" (0-2 m/s), "Turn Rate" (0-1 rad/s)

4. **Status Panel**:
   - Text labels: "Battery: 85%", "Connection: OK"

### Teleoperation Controller Script

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using UnityEngine.UI;

public class TeleoperationController : MonoBehaviour
{
    private ROSConnection ros;

    public Slider speedSlider;
    public Slider turnRateSlider;
    public Text statusText;

    private float maxSpeed = 2.0f;
    private float maxTurnRate = 1.0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>("/cmd_vel");

        statusText.text = "Status: Connected";
    }

    void Update()
    {
        // Read keyboard input
        float forward = Input.GetAxis("Vertical");   // W/S or Up/Down
        float turn = Input.GetAxis("Horizontal");    // A/D or Left/Right

        if (forward != 0 || turn != 0)
        {
            SendVelocityCommand(forward, turn);
        }
    }

    void SendVelocityCommand(float forward, float turn)
    {
        TwistMsg msg = new TwistMsg();

        // Apply slider scaling
        msg.linear.x = forward * speedSlider.value;
        msg.angular.z = turn * turnRateSlider.value;

        ros.Publish("/cmd_vel", msg);
    }

    public void OnEmergencyStop()
    {
        // Stop robot immediately
        TwistMsg msg = new TwistMsg();
        msg.linear.x = 0;
        msg.angular.z = 0;

        ros.Publish("/cmd_vel", msg);
        statusText.text = "Status: EMERGENCY STOP";
        statusText.color = Color.red;

        Debug.Log("EMERGENCY STOP ACTIVATED");
    }

    public void OnResetEmergencyStop()
    {
        statusText.text = "Status: Connected";
        statusText.color = Color.green;
    }
}
```

**Setup:**
1. Attach script to a GameObject
2. Assign UI elements in Inspector (sliders, text)
3. Connect buttons to `OnEmergencyStop` and `OnResetEmergencyStop`

**Run**: Control the robot with WASD keys, adjust speed with sliders, emergency stop with button.

---

## Key Takeaways

1. **Unity excels at visualization and UI** - Use it for demos, teleoperation, and human interaction
2. **ROS-TCP-Connector bridges Unity and ROS 2** - Seamless message passing over TCP
3. **Coordinate systems differ** - Always convert ROS (Z-up) to Unity (Y-up)
4. **Articulation Bodies simulate robot joints** - Unity's physics for robot arms
5. **Humanoid Animator enables lifelike motion** - Perfect for humanoid robots
6. **Build interactive dashboards** - Camera feeds, controls, status displays

---

## Exercises

### Conceptual

1. **Gazebo vs Unity trade-offs**: You need to test a path planning algorithm AND create a customer demo. Which tool(s) do you use for each, and why?

### Coding

2. **LIDAR Visualization**: Subscribe to `/robot/scan` (LaserScan) and draw LIDAR rays in Unity using `LineRenderer` components. Color rays red if obstacle is within 1m.

3. **Gamepad Control**: Modify the teleoperation controller to support Xbox/PlayStation gamepad input (use `Input.GetAxis("LeftStickVertical")` and `Input.GetAxis("RightStickHorizontal")`).

4. **Humanoid Mirroring**: Create a Unity humanoid that mirrors a real humanoid's joint states in real-time (subscribe to `/joint_states`, map to Unity's Animator bones).

---

## Next: Module 3 - NVIDIA Isaac Sim

In Weeks 8-10, we'll explore **NVIDIA Isaac Sim**, a simulation platform that combines:
- **Photorealistic rendering** (better than Unity)
- **Accurate physics** (PhysX, better than Gazebo)
- **AI integration** (reinforcement learning, synthetic data generation)

See you in [Module 3](../module-3/intro)!

---

**Further Resources:**
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS-TCP-Connector Docs](https://github.com/Unity-Technologies/ROS-TCP-Connector)
- [URDF Importer Tutorial](https://github.com/Unity-Technologies/URDF-Importer/blob/main/com.unity.robotics.urdf-importer/Documentation~/index.md)
- [Unity Learn - Robotics](https://learn.unity.com/course/unity-robotics-hub)
