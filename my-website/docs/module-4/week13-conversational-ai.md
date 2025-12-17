---
sidebar_position: 2
title: Week 13 - Conversational AI Integration
---

# Week 13: Conversational AI Integration for Humanoid Robots

**Learning Objectives:**
- Understand the voice-to-action pipeline for robotic systems
- Integrate speech recognition with OpenAI Whisper
- Use LLMs (GPT-4, Claude) for task planning and intent parsing
- Design action schemas for executable robot commands
- Implement safety validation and human-in-the-loop confirmation
- Combine voice commands with vision for multimodal interaction

---

## The Vision: Conversational Robots

Imagine saying to a humanoid robot:
> "Go to the kitchen, grab the red mug from the counter, and bring it to me."

The robot must:
1. **Understand** your speech (speech recognition)
2. **Parse intent** (natural language understanding)
3. **Plan actions** (task decomposition)
4. **Execute** (navigate, grasp, deliver)
5. **Handle failures** (ask clarifications, retry)

This is the **voice-to-action pipeline**, and it's becoming real with modern AI.

---

## The Voice-to-Action Pipeline

```
[Human Speech]
    ↓
[Speech Recognition] (Whisper)
    ↓
[Text Command: "Grab the red mug"]
    ↓
[LLM Task Planner] (GPT-4/Claude)
    ↓
[Action Sequence: navigate(kitchen) → detect(red mug) → grasp() → navigate(user)]
    ↓
[Safety Validation] (workspace bounds, collision check)
    ↓
[ROS 2 Execution] (MoveIt2, navigation actions)
    ↓
[Robot performs task]
```

**Key Components:**
1. **Speech Recognition**: Audio → Text (Whisper, Vosk, Google Speech-to-Text)
2. **Language Model**: Text → Structured Actions (GPT-4, Claude, LLaMA)
3. **Action Executor**: Actions → Robot Commands (ROS 2 actions/services)
4. **Vision (optional)**: Grounding ("the red mug" → pixel location)
5. **Safety Validator**: Reject dangerous commands

---

## Speech Recognition with OpenAI Whisper

**Whisper** is OpenAI's open-source, state-of-the-art speech recognition model.

**Why Whisper?**
- ✅ **Multilingual**: 99 languages (English, Spanish, Chinese, etc.)
- ✅ **Robust**: Handles accents, background noise
- ✅ **Open-source**: Free, runs locally (no API costs)
- ✅ **Fast**: Real-time capable on GPU

### Installation

```bash
pip install openai-whisper torch torchaudio
```

### Basic Usage

```python
import whisper

# Load model (options: tiny, base, small, medium, large)
model = whisper.load_model("base")  # 74M parameters, good balance

# Transcribe audio file
result = model.transcribe("audio.mp3")

print(result["text"])
# Output: "Go to the kitchen and grab the red mug."
```

**Model Sizes:**

| Model | Parameters | Speed | Accuracy | Use Case |
|-------|-----------|-------|----------|----------|
| `tiny` | 39M | Very fast | Good | Real-time, edge devices |
| `base` | 74M | Fast | Better | General use |
| `small` | 244M | Medium | Great | High accuracy needed |
| `medium` | 769M | Slow | Excellent | Low latency not critical |
| `large` | 1550M | Very slow | Best | Offline, batch processing |

**Recommendation**: Start with `base` for real-time robotics.

### Real-Time Transcription from Microphone

```python
import whisper
import pyaudio
import wave
import tempfile
import os

class RealtimeWhisper:
    def __init__(self, model_size="base"):
        self.model = whisper.load_model(model_size)
        self.audio = pyaudio.PyAudio()

        # Audio settings
        self.sample_rate = 16000
        self.chunk_size = 1024
        self.format = pyaudio.paInt16
        self.channels = 1

    def record_audio(self, duration=5):
        """Record audio from microphone"""
        stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )

        print(f"🎤 Recording for {duration} seconds...")
        frames = []

        for _ in range(0, int(self.sample_rate / self.chunk_size * duration)):
            data = stream.read(self.chunk_size)
            frames.append(data)

        print("✅ Recording complete")

        stream.stop_stream()
        stream.close()

        # Save to temporary file
        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".wav")
        wf = wave.open(temp_file.name, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(self.audio.get_sample_size(self.format))
        wf.setframerate(self.sample_rate)
        wf.writeframes(b''.join(frames))
        wf.close()

        return temp_file.name

    def transcribe(self, audio_file=None, duration=5):
        """Transcribe audio to text"""
        if audio_file is None:
            audio_file = self.record_audio(duration)

        result = self.model.transcribe(audio_file, language="en")

        # Cleanup temp file
        if audio_file.startswith(tempfile.gettempdir()):
            os.remove(audio_file)

        return result["text"]

# Example usage
if __name__ == "__main__":
    whisper_system = RealtimeWhisper()

    while True:
        text = whisper_system.transcribe(duration=5)
        print(f"You said: {text}")

        if "stop" in text.lower():
            print("Exiting...")
            break
```

**Usage:**
```bash
python realtime_whisper.py
# Speak after "Recording..." prompt
# Output: "You said: Go to the kitchen and grab the red mug."
```

---

## LLM Task Planning with GPT-4

Once we have text, we need to convert it to **structured actions** the robot can execute.

### Defining Action Schema

```python
from dataclasses import dataclass
from typing import Optional
from enum import Enum

class ActionType(Enum):
    NAVIGATE = "navigate"
    PICK = "pick"
    PLACE = "place"
    WAIT = "wait"
    SPEAK = "speak"

@dataclass
class RobotAction:
    """Structured robot action"""
    action_type: ActionType
    object: Optional[str] = None
    location: Optional[str] = None
    duration_sec: Optional[int] = None
    message: Optional[str] = None

    def to_dict(self):
        return {
            "action": self.action_type.value,
            "object": self.object,
            "location": self.location,
            "duration": self.duration_sec,
            "message": self.message
        }

# Example actions
actions = [
    RobotAction(ActionType.NAVIGATE, location="kitchen"),
    RobotAction(ActionType.PICK, object="red mug", location="counter"),
    RobotAction(ActionType.NAVIGATE, location="user"),
    RobotAction(ActionType.PLACE, object="red mug", location="table"),
    RobotAction(ActionType.SPEAK, message="Here is your mug!")
]
```

### GPT-4 Task Planner

```python
import openai
import json
from typing import List

class GPT4TaskPlanner:
    def __init__(self, api_key: str):
        openai.api_key = api_key

        self.system_prompt = """You are a robot task planner. Convert natural language commands into structured JSON action sequences.

Available actions:
- navigate(location: str) - Move to a location (e.g., "kitchen", "living room")
- pick(object: str, location: str) - Grasp an object at a location
- place(object: str, location: str) - Put down an object at a location
- wait(duration_sec: int) - Wait for specified seconds
- speak(message: str) - Say a message

Rules:
1. Break complex tasks into simple steps
2. Navigate before picking/placing objects
3. If location is ambiguous, use "unknown" and robot will ask for clarification
4. Always confirm completion with a speak action

Output format (JSON array):
[
  {"action": "navigate", "location": "kitchen"},
  {"action": "pick", "object": "red mug", "location": "counter"},
  {"action": "navigate", "location": "user"},
  {"action": "place", "object": "red mug", "location": "table"}
]
"""

    def plan_task(self, command: str, context: str = "") -> List[RobotAction]:
        """Convert natural language command to action sequence"""

        user_message = f"Command: {command}"
        if context:
            user_message += f"\nContext: {context}"

        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": user_message}
            ],
            temperature=0.0  # Deterministic output
        )

        # Parse JSON response
        action_json = json.loads(response.choices[0].message.content)

        # Convert to RobotAction objects
        actions = []
        for item in action_json:
            action_type = ActionType(item["action"])
            actions.append(RobotAction(
                action_type=action_type,
                object=item.get("object"),
                location=item.get("location"),
                duration_sec=item.get("duration"),
                message=item.get("message")
            ))

        return actions

# Example usage
if __name__ == "__main__":
    planner = GPT4TaskPlanner(api_key="your-api-key")

    command = "Go to the kitchen, grab the red mug, and bring it to me."
    actions = planner.plan_task(command)

    print("Generated Action Plan:")
    for i, action in enumerate(actions, 1):
        print(f"{i}. {action.to_dict()}")

# Output:
# Generated Action Plan:
# 1. {'action': 'navigate', 'location': 'kitchen', ...}
# 2. {'action': 'pick', 'object': 'red mug', 'location': 'counter', ...}
# 3. {'action': 'navigate', 'location': 'user', ...}
# 4. {'action': 'place', 'object': 'red mug', 'location': 'table', ...}
# 5. {'action': 'speak', 'message': 'Here is your red mug!', ...}
```

### Alternative: Claude API

If you prefer Anthropic's Claude:

```python
import anthropic

class ClaudeTaskPlanner:
    def __init__(self, api_key: str):
        self.client = anthropic.Client(api_key=api_key)
        self.system_prompt = """[Same system prompt as GPT-4]"""

    def plan_task(self, command: str, context: str = "") -> List[RobotAction]:
        message = self.client.messages.create(
            model="claude-3-sonnet-20240229",
            max_tokens=1024,
            system=self.system_prompt,
            messages=[
                {"role": "user", "content": f"Command: {command}\n{context}"}
            ]
        )

        # Parse JSON response
        action_json = json.loads(message.content[0].text)

        # Convert to RobotAction objects (same as above)
        actions = []
        for item in action_json:
            actions.append(RobotAction(
                action_type=ActionType(item["action"]),
                object=item.get("object"),
                location=item.get("location"),
                duration_sec=item.get("duration"),
                message=item.get("message")
            ))

        return actions
```

**Claude advantages**: Longer context, better at following instructions, more affordable for large volumes.

---

## Intent Parsing and Action Execution

### Simple Intent Parser (Regex + LLM)

For simpler cases, you can use regex + keyword matching:

```python
import re
from typing import List, Optional

class IntentParser:
    def __init__(self):
        # Define location keywords
        self.locations = {
            "kitchen", "living room", "bedroom", "bathroom",
            "office", "garage", "dining room"
        }

        # Define object keywords
        self.objects = {
            "mug", "cup", "plate", "bottle", "book", "phone",
            "pen", "remote", "keys", "glasses"
        }

    def parse_navigate(self, text: str) -> Optional[RobotAction]:
        """Detect navigation commands"""
        patterns = [
            r"go to (?:the )?(\w+)",
            r"move to (?:the )?(\w+)",
            r"navigate to (?:the )?(\w+)"
        ]

        for pattern in patterns:
            match = re.search(pattern, text, re.IGNORECASE)
            if match:
                location = match.group(1).lower()
                if location in self.locations:
                    return RobotAction(ActionType.NAVIGATE, location=location)

        return None

    def parse_pick(self, text: str) -> Optional[RobotAction]:
        """Detect pick/grasp commands"""
        patterns = [
            r"(?:pick up|grab|get|take) (?:the )?(\w+(?: \w+)?)",
            r"bring me (?:the )?(\w+(?: \w+)?)"
        ]

        for pattern in patterns:
            match = re.search(pattern, text, re.IGNORECASE)
            if match:
                obj = match.group(1).lower()
                # Extract location if mentioned
                location = "unknown"
                if "from" in text:
                    loc_match = re.search(r"from (?:the )?(\w+)", text, re.IGNORECASE)
                    if loc_match:
                        location = loc_match.group(1).lower()

                return RobotAction(ActionType.PICK, object=obj, location=location)

        return None

    def parse(self, text: str) -> List[RobotAction]:
        """Parse text into actions"""
        actions = []

        # Try each parser
        if action := self.parse_navigate(text):
            actions.append(action)

        if action := self.parse_pick(text):
            actions.append(action)

        return actions

# Example usage
parser = IntentParser()

commands = [
    "Go to the kitchen",
    "Pick up the red mug from the counter",
    "Bring me the book from the office"
]

for cmd in commands:
    actions = parser.parse(cmd)
    print(f"Command: {cmd}")
    print(f"Actions: {[a.to_dict() for a in actions]}\n")
```

**When to use regex vs LLM:**
- **Regex**: Fixed command set, low latency, no API costs
- **LLM**: Complex/ambiguous commands, natural phrasing, context-aware

---

## ROS 2 Action Executor

Now let's execute actions on a real robot.

### Action Executor Class

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
import time

class RobotActionExecutor(Node):
    def __init__(self):
        super().__init__('robot_action_executor')

        # Navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Speech publisher (text-to-speech)
        self.speech_pub = self.create_publisher(String, '/robot_speech', 10)

        # Predefined locations
        self.locations = {
            "kitchen": (5.0, 2.0, 0.0),
            "living room": (0.0, 0.0, 0.0),
            "bedroom": (10.0, 5.0, 1.57)
        }

    def execute_action(self, action: RobotAction) -> bool:
        """Execute a single action"""
        self.get_logger().info(f"Executing: {action.to_dict()}")

        if action.action_type == ActionType.NAVIGATE:
            return self.navigate_to(action.location)

        elif action.action_type == ActionType.PICK:
            return self.pick_object(action.object, action.location)

        elif action.action_type == ActionType.PLACE:
            return self.place_object(action.object, action.location)

        elif action.action_type == ActionType.WAIT:
            time.sleep(action.duration_sec)
            return True

        elif action.action_type == ActionType.SPEAK:
            return self.speak(action.message)

        return False

    def navigate_to(self, location: str) -> bool:
        """Navigate to named location"""
        if location not in self.locations:
            self.get_logger().error(f"Unknown location: {location}")
            return False

        x, y, theta = self.locations[location]

        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = theta

        # Wait for action server
        self.nav_client.wait_for_server()

        # Send goal
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        return True

    def pick_object(self, obj: str, location: str) -> bool:
        """Pick object (stub - integrate with MoveIt2 or grasp planner)"""
        self.get_logger().info(f"Picking {obj} from {location}")

        # TODO: Integrate with vision + MoveIt2
        # 1. Detect object in camera (YOLO, etc.)
        # 2. Compute grasp pose
        # 3. Execute MoveIt2 pick action

        time.sleep(2.0)  # Simulate grasping
        self.speak(f"Picked up {obj}")
        return True

    def place_object(self, obj: str, location: str) -> bool:
        """Place object (stub)"""
        self.get_logger().info(f"Placing {obj} at {location}")

        time.sleep(2.0)  # Simulate placing
        self.speak(f"Placed {obj}")
        return True

    def speak(self, message: str) -> bool:
        """Speak message (publish to TTS node)"""
        msg = String()
        msg.data = message
        self.speech_pub.publish(msg)
        self.get_logger().info(f"Speaking: {message}")
        return True

    def execute_plan(self, actions: List[RobotAction]) -> bool:
        """Execute full action sequence"""
        for action in actions:
            success = self.execute_action(action)
            if not success:
                self.get_logger().error(f"Action failed: {action}")
                return False

        self.get_logger().info("Plan executed successfully!")
        return True

# Example usage
def main(args=None):
    rclpy.init(args=args)
    executor = RobotActionExecutor()

    # Example action plan
    plan = [
        RobotAction(ActionType.NAVIGATE, location="kitchen"),
        RobotAction(ActionType.PICK, object="mug", location="counter"),
        RobotAction(ActionType.NAVIGATE, location="living room"),
        RobotAction(ActionType.PLACE, object="mug", location="table"),
        RobotAction(ActionType.SPEAK, message="Task complete!")
    ]

    executor.execute_plan(plan)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Complete Voice-Controlled Robot

Putting it all together:

```python
import rclpy
from rclpy.node import Node

class VoiceControlledRobot(Node):
    def __init__(self):
        super().__init__('voice_controlled_robot')

        # Initialize components
        self.whisper = RealtimeWhisper(model_size="base")
        self.planner = GPT4TaskPlanner(api_key="your-api-key")
        self.executor = RobotActionExecutor()
        self.safety_validator = SafetyValidator()

    def run(self):
        """Main voice control loop"""
        self.get_logger().info("Voice control started. Say 'stop robot' to exit.")

        while rclpy.ok():
            # 1. Listen to user
            self.get_logger().info("🎤 Listening...")
            text = self.whisper.transcribe(duration=5)
            self.get_logger().info(f"You said: {text}")

            # Exit condition
            if "stop robot" in text.lower():
                self.get_logger().info("Exiting voice control.")
                break

            # 2. Plan actions
            try:
                actions = self.planner.plan_task(text)
                self.get_logger().info(f"Plan: {[a.to_dict() for a in actions]}")
            except Exception as e:
                self.get_logger().error(f"Planning failed: {e}")
                self.executor.speak("Sorry, I didn't understand that command.")
                continue

            # 3. Validate safety
            safe, reason = self.safety_validator.validate_plan(actions)
            if not safe:
                self.get_logger().warn(f"Unsafe plan: {reason}")
                self.executor.speak(f"Cannot execute: {reason}")
                continue

            # 4. Confirm with user
            self.executor.speak("Should I proceed?")
            confirmation = self.whisper.transcribe(duration=3)

            if "yes" not in confirmation.lower():
                self.executor.speak("Okay, canceling.")
                continue

            # 5. Execute
            self.executor.speak("Executing now.")
            success = self.executor.execute_plan(actions)

            if success:
                self.executor.speak("Task completed successfully!")
            else:
                self.executor.speak("Task failed. Please try again.")

def main(args=None):
    rclpy.init(args=args)
    robot = VoiceControlledRobot()
    robot.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Usage:**
```bash
ros2 run my_robot voice_controlled_robot

# Output:
# 🎤 Listening...
# You said: Go to the kitchen and grab the red mug.
# Plan: [{'action': 'navigate', 'location': 'kitchen'}, ...]
# Speaking: Should I proceed?
# 🎤 Listening...
# You said: Yes.
# Speaking: Executing now.
# [Robot performs actions]
# Speaking: Task completed successfully!
```

---

## Multimodal: Vision + Voice

To ground "the red mug" in the real world, we need **vision**.

### Object Detection with YOLO

```python
from ultralytics import YOLO
import cv2
import numpy as np

class ObjectGrounder:
    def __init__(self):
        # Load YOLOv8 model
        self.model = YOLO("yolov8n.pt")

    def detect_objects(self, image_path: str):
        """Detect objects in image"""
        results = self.model(image_path)

        objects = []
        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                label = result.names[cls_id]
                confidence = float(box.conf[0])
                bbox = box.xyxy[0].tolist()  # [x1, y1, x2, y2]

                objects.append({
                    'label': label,
                    'confidence': confidence,
                    'bbox': bbox
                })

        return objects

    def find_object(self, image_path: str, query: str):
        """Find specific object by description"""
        objects = self.detect_objects(image_path)

        # Simple matching (extend with color detection, etc.)
        for obj in objects:
            if query.lower() in obj['label'].lower():
                return obj

        return None

# Example usage
grounder = ObjectGrounder()

# Detect all objects
objects = grounder.detect_objects("robot_camera.jpg")
print(f"Detected: {[obj['label'] for obj in objects]}")

# Find specific object
mug = grounder.find_object("robot_camera.jpg", query="mug")
if mug:
    print(f"Found mug at {mug['bbox']} with confidence {mug['confidence']:.2f}")
```

**Integration with robot**:
1. User says: "Pick up the red mug"
2. LLM generates: `pick(object="red mug", location="counter")`
3. Vision detects red mug at pixel (320, 240)
4. Convert pixel → 3D pose (using depth camera + camera intrinsics)
5. Execute MoveIt2 grasp at that pose

---

## Safety Validation

Always validate commands before execution!

```python
class SafetyValidator:
    def __init__(self):
        # Define dangerous objects
        self.dangerous_objects = ["knife", "scissors", "fire", "chemical"]

        # Define restricted locations
        self.restricted_areas = ["stairs", "pool", "street"]

        # Workspace bounds (x_min, x_max, y_min, y_max)
        self.workspace = (-10.0, 10.0, -5.0, 5.0)

    def validate_plan(self, actions: List[RobotAction]) -> tuple[bool, str]:
        """Validate action plan for safety"""

        for action in actions:
            # Check dangerous objects
            if action.action_type in [ActionType.PICK, ActionType.PLACE]:
                if action.object:
                    for dangerous in self.dangerous_objects:
                        if dangerous in action.object.lower():
                            return False, f"Cannot manipulate dangerous object: {action.object}"

            # Check restricted areas
            if action.action_type == ActionType.NAVIGATE:
                for restricted in self.restricted_areas:
                    if restricted in action.location.lower():
                        return False, f"Cannot navigate to restricted area: {action.location}"

            # Check workspace bounds (if location has coordinates)
            # ... (add coordinate checking here)

        return True, "Plan is safe"

    def check_action_safety(self, action: RobotAction) -> tuple[bool, str]:
        """Check single action safety"""
        return self.validate_plan([action])
```

**Safety checklist:**
- ✅ No dangerous objects (knives, chemicals)
- ✅ No restricted areas (stairs, pools)
- ✅ Workspace bounds enforced
- ✅ Collision checking (integrate with MoveIt2)
- ✅ Human confirmation for critical actions

---

## Key Takeaways

1. **Voice-to-action pipeline**: Speech recognition → LLM planning → ROS 2 execution
2. **Whisper for speech recognition**: Open-source, multilingual, robust
3. **LLMs as task planners**: GPT-4/Claude convert natural language to structured actions
4. **Action schemas**: Define clear robot capabilities (navigate, pick, place, speak)
5. **Safety validation is critical**: Always check for dangerous commands before execution
6. **Multimodal integration**: Combine voice + vision for grounded understanding
7. **Human-in-the-loop**: Confirm critical actions before execution
8. **Modular design**: Separate speech, planning, execution, and safety layers

---

## Exercises

### Conceptual

1. **Safety scenarios**: List 5 voice commands that should be rejected by a safety validator, and why.

2. **Error handling**: The robot fails to find "the red mug" in the kitchen. Design a dialog flow for asking the user for clarification.

### Coding

3. **Color detection**: Extend `ObjectGrounder` to detect object colors (e.g., "red mug" vs "blue mug") using OpenCV color masks.

4. **Context-aware planning**: Modify `GPT4TaskPlanner` to maintain conversation history, so follow-up commands like "bring me another one" work correctly.

5. **Complete humanoid voice control**: Integrate Weeks 11-12 (kinematics, locomotion) + Week 13 (voice) to create a voice-controlled humanoid that can walk to locations and pick up objects.

---

## Next: Final Project

In the final weeks, you'll integrate everything:
- **ROS 2 fundamentals** (Module 1)
- **Simulation & visualization** (Module 2)
- **NVIDIA Isaac Sim** (Module 3)
- **Conversational AI** (Module 4)

**Project ideas:**
- 🤖 Voice-controlled warehouse robot
- 🏠 Home assistant humanoid
- 🎓 Educational robot tutor
- 🏥 Medical delivery robot

Congratulations on completing the conversational AI module! You now have the tools to build truly interactive robotic systems.

---

**Further Resources:**
- [OpenAI Whisper](https://github.com/openai/whisper)
- [GPT-4 API Docs](https://platform.openai.com/docs/guides/gpt)
- [Claude API Docs](https://docs.anthropic.com/claude/reference/getting-started-with-the-api)
- [YOLOv8](https://github.com/ultralytics/ultralytics)
- [ROS 2 Navigation](https://navigation.ros.org/)
