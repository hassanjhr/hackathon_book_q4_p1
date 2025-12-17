---
sidebar_position: 4
title: Week 10 - Reinforcement Learning Basics
---

# Week 10: Reinforcement Learning Basics

Welcome to Week 10! This week, you'll learn the fundamentals of **Reinforcement Learning (RL)** for robotics. RL enables robots to learn complex behaviors through trial and error, without explicit programming. You'll understand the sim-to-real problem, design reward functions, and train policies using GPU-accelerated simulation.

By the end of this week, you'll have trained a robot policy using Stable-Baselines3 and understand how to bridge the simulation-reality gap.

---

## Learning Objectives

By the end of this week, you will be able to:

1. Understand Reinforcement Learning (RL) fundamentals (MDP, policy, value)
2. Learn the sim-to-real transfer problem and solutions
3. Understand Isaac Gym for GPU-accelerated RL training
4. Implement a simple RL environment for a robotic task
5. Design reward functions for robot behaviors
6. Train a basic policy with Stable-Baselines3

---

## What is Reinforcement Learning?

### Definition

**Reinforcement Learning (RL)** is a machine learning paradigm where an agent learns **what actions to take** in an environment to **maximize cumulative reward**.

### Key Difference from Supervised Learning

| Aspect | Supervised Learning | Reinforcement Learning |
|--------|---------------------|------------------------|
| **Training Data** | (input, correct output) pairs | (state, action, reward, next state) sequences |
| **Feedback** | Immediate (correct label) | Delayed (reward after many actions) |
| **Exploration** | None (passive learning) | Active (agent chooses actions) |
| **Goal** | Minimize prediction error | Maximize cumulative reward |

**Example:**
- **Supervised**: "This image is a cat" → "Correct!"
- **RL**: "Move arm forward" → "Distance to object decreased" → +0.1 reward

---

### Robotics Applications

1. **Manipulation**
   - Grasping objects (learning hand-eye coordination)
   - Assembly tasks (inserting pegs into holes)
   - Tool use (using hammers, screwdrivers)

2. **Locomotion**
   - Walking, running, jumping (humanoid robots)
   - Quadruped gait optimization
   - Obstacle navigation

3. **Navigation**
   - Autonomous driving (path planning, obstacle avoidance)
   - Drone flight control
   - Warehouse navigation

4. **Task Planning**
   - Multi-step goal completion
   - Household chores (cleaning, organizing)

---

### Why RL for Robotics?

**Advantages:**
- ✅ **No manual programming**: Learns from trial and error
- ✅ **Handles high dimensions**: Works with raw sensor data (cameras, LIDAR)
- ✅ **Adapts to environments**: Can generalize to unseen scenarios
- ✅ **Discovers creative solutions**: May find strategies humans didn't consider

**Challenges:**
- ❌ **Sample inefficient**: Needs thousands/millions of trials
- ❌ **Sim-to-real gap**: What works in simulation may fail in reality
- ❌ **Reward engineering**: Hard to design good reward functions
- ❌ **Safety**: Exploration can be dangerous on real hardware

---

## RL Fundamentals: The MDP Framework

### Markov Decision Process (MDP)

An MDP is a mathematical framework for modeling decision-making. It consists of:

1. **State (s)**: The robot's current situation
   - Example: `[joint_angles, end_effector_position, object_position]`

2. **Action (a)**: What the robot can do
   - Example: `[joint1_torque, joint2_torque, ..., joint_n_torque]`

3. **Reward (r)**: Scalar feedback signal
   - Example: `+10.0` for grasping object, `-0.01` per timestep (time penalty)

4. **Transition (s' | s, a)**: Probability of reaching next state `s'` after taking action `a` in state `s`
   - In deterministic environments: next state is fully determined by current state and action

5. **Policy (π)**: Strategy mapping states to actions
   - Example: π(s) = a (deterministic policy)
   - Example: π(a | s) = probability distribution over actions (stochastic policy)

---

### Goal: Find Optimal Policy

The goal of RL is to find the **optimal policy π\*** that maximizes **expected cumulative reward**:

```
π* = argmax_π E[Σ γ^t * r_t]
```

Where:
- **γ (gamma)**: Discount factor (0 < γ < 1)
  - γ = 0.99: Value future rewards at 99% of immediate rewards
  - γ = 0.0: Only care about immediate reward (myopic)
  - γ = 1.0: All rewards weighted equally (can diverge)

**Intuition:** We want to maximize **total reward** over the episode, but prioritize near-term rewards slightly more than distant rewards.

---

### Example: Robot Reaching Task

**Scenario:** 2-DOF robot arm must reach a target position.

```python
State: [joint1_angle, joint2_angle, target_x, target_y]
# Example: [0.5, -0.3, 1.2, 0.8]

Action: [joint1_torque, joint2_torque]
# Example: [0.2, -0.1]  (normalized -1 to 1)

Reward:
  - Dense: -distance(end_effector, target)  # Closer = higher reward
  - Sparse: +10.0 if distance < 0.1m, else 0.0
  - Time penalty: -0.01 per timestep (encourages fast reaching)
```

**Optimal Policy:** The arm learns to move joints such that the end-effector reaches the target in the fewest steps.

---

## The Sim-to-Real Problem

### Reality Gap

Policies trained in simulation often **fail on real robots** due to discrepancies between sim and reality:

1. **Physics Inaccuracies**
   - Friction models (sim uses constant friction, reality varies)
   - Contact dynamics (bouncing, sliding)
   - Soft body deformation (cables, cloth)

2. **Sensor Noise**
   - Real cameras have noise, motion blur, lens distortion
   - LIDAR has dropout points, reflections
   - IMUs drift over time

3. **Actuator Delays**
   - Real motors have ~10-50ms latency
   - Backlash in gears (position errors)
   - Non-linear torque curves

4. **Unmodeled Effects**
   - Cable drag (tethered robots)
   - Wear and tear (joint friction increases)
   - Temperature effects (battery voltage drops)

**Result:** A policy that works perfectly in Isaac Gym (1000 successful grasps) may fail 80% of the time on the real robot.

---

### Solutions: Bridging the Sim-to-Real Gap

#### 1. Domain Randomization (DR)

Randomize simulation parameters during training to force the policy to be **robust** to variations.

**Randomizable Parameters:**

```python
# Physics randomization
mass = random.uniform(0.05, 0.5)  # 50g to 500g
friction = random.uniform(0.2, 0.8)
damping = random.uniform(0.01, 0.1)

# Visual randomization (for vision-based policies)
lighting_intensity = random.uniform(500, 5000)
texture_color = random.uniform((0, 0, 0), (1, 1, 1))  # RGB

# Sensor noise
camera_noise = np.random.normal(0, 0.02, image.shape)
joint_angle_noise = np.random.normal(0, 0.01, num_joints)
```

**Intuition:** If the policy works across a wide range of sim parameters, it's more likely to work on the real robot (which is just another "variant" in the randomized distribution).

---

#### 2. System Identification

Measure **real robot parameters** and tune the simulation to match.

**Process:**
1. Run experiments on real robot (e.g., drop object 100 times, measure bounce height)
2. Measure friction coefficients (drag object on surface, measure force)
3. Measure actuator response times (command torque, measure delay)
4. Tune simulation parameters to match measurements

**Challenge:** Time-consuming, requires access to hardware, may not capture all effects.

---

#### 3. Sim-to-Sim-to-Real

Train in multiple simulators with increasing realism:

```
Step 1: Train in simple sim (Isaac Gym - fast, 1000x speedup)
  ↓
Step 2: Fine-tune in photorealistic sim (Isaac Sim - realistic physics/rendering)
  ↓
Step 3: Deploy to real robot (with optional fine-tuning on real data)
```

---

#### 4. Residual RL

Train a **base policy** in simulation, then fine-tune a **residual policy** on the real robot.

```python
action = base_policy(state) + residual_policy(state)
```

**Advantage:** Only need ~100-1000 real samples (vs millions for training from scratch).

---

## Isaac Gym: Massively Parallel RL Training

### What is Isaac Gym?

**Isaac Gym** is NVIDIA's GPU-accelerated physics simulator designed specifically for **RL training**. Unlike traditional simulators (MuJoCo, PyBullet) that run one environment at a time on CPU, Isaac Gym simulates **thousands of parallel environments** on a single GPU.

### Key Features

1. **Massively Parallel Simulation**
   - Simulate 4096+ environments simultaneously on one GPU
   - 1000x faster than CPU-based methods

2. **End-to-End GPU**
   - Physics simulation on GPU
   - Neural network training on GPU
   - No CPU↔GPU transfers (PyTorch tensors stay on GPU)

3. **Integrated with PyTorch**
   - Observations are PyTorch tensors
   - Actions are PyTorch tensors
   - Seamless integration with RL libraries

---

### Performance Comparison

| Aspect | CPU-based (MuJoCo) | Isaac Gym (GPU) |
|--------|-------------------|-----------------|
| **Environments/GPU** | 1 | 4096+ |
| **Training Time** (humanoid walk) | 48 hours (1M samples) | **30 minutes** |
| **Hardware** | 32-core CPU | Single RTX 4090 |
| **Framework** | OpenAI Gym | PyTorch native |
| **Sim FPS** | 100-1000 | 100,000+ (aggregate) |

**Example:** Training a humanoid to walk from scratch:
- **CPU (MuJoCo + Stable-Baselines3)**: 48 hours
- **Isaac Gym (4096 parallel envs)**: 30 minutes

---

### Architecture

```
┌─────────────────────────────────────┐
│     Isaac Gym (PhysX on GPU)        │
│  (4096 parallel environments)       │
├─────────────────────────────────────┤
│    Observations (PyTorch tensors)   │
│         ↓ (directly on GPU)         │
│   Neural Network (policy/value)     │
│         ↓ (actions)                 │
│   Isaac Gym (apply actions, step)   │
└─────────────────────────────────────┘
```

**No CPU↔GPU transfers:** Everything stays on GPU, enabling maximum throughput.

---

## Simple RL Environment: Reaching Task

Let's implement a simple 2-DOF robot arm reaching task using the OpenAI Gym API.

### Environment Code

```python
"""
Simple Reaching Environment: 2-DOF robot arm reaching a target.
"""
import numpy as np
import gym
from gym import spaces

class ReachingEnv(gym.Env):
    """2-DOF robot arm reaching environment"""

    def __init__(self):
        super().__init__()

        # State: [joint1_angle, joint2_angle, target_x, target_y]
        self.observation_space = spaces.Box(
            low=np.array([-np.pi, -np.pi, -2, -2]),
            high=np.array([np.pi, np.pi, 2, 2]),
            dtype=np.float32
        )

        # Action: [joint1_torque, joint2_torque] (normalized -1 to 1)
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(2,),
            dtype=np.float32
        )

        # Robot parameters
        self.link1_length = 1.0  # meters
        self.link2_length = 1.0  # meters

        self.timestep = 0
        self.reset()

    def reset(self):
        """Reset environment to initial state"""
        # Random initial joint angles
        self.joint_angles = np.random.uniform(-np.pi/2, np.pi/2, 2)

        # Random target position (reachable workspace)
        self.target = np.random.uniform(-1.5, 1.5, 2)

        self.timestep = 0

        return self._get_obs()

    def _get_obs(self):
        """Get current observation (state)"""
        return np.concatenate([self.joint_angles, self.target]).astype(np.float32)

    def _forward_kinematics(self):
        """Compute end-effector position from joint angles"""
        q1, q2 = self.joint_angles

        # Planar 2-link forward kinematics
        x = self.link1_length * np.cos(q1) + \
            self.link2_length * np.cos(q1 + q2)
        y = self.link1_length * np.sin(q1) + \
            self.link2_length * np.sin(q1 + q2)

        return np.array([x, y])

    def step(self, action):
        """Take a step in the environment"""
        # Apply torques (simplified dynamics: direct angle change)
        self.joint_angles += action * 0.1  # Scale action

        # Clip to joint limits
        self.joint_angles = np.clip(self.joint_angles, -np.pi, np.pi)

        # Compute end-effector position
        ee_pos = self._forward_kinematics()

        # Compute reward (negative distance to target)
        distance = np.linalg.norm(ee_pos - self.target)
        reward = -distance

        # Success bonus
        done = False
        if distance < 0.1:
            reward += 10.0
            done = True

        # Max episode length (prevent infinite episodes)
        self.timestep += 1
        if self.timestep > 100:
            done = True

        return self._get_obs(), reward, done, {}

    def render(self, mode='human'):
        """Render environment (optional)"""
        pass  # Implement visualization if needed

# Test environment
if __name__ == '__main__':
    env = ReachingEnv()
    obs = env.reset()

    print(f"Initial obs: {obs}")

    for i in range(100):
        action = env.action_space.sample()  # Random action
        obs, reward, done, info = env.step(action)

        print(f"Step {i}: reward={reward:.3f}, done={done}")

        if done:
            print("Episode finished!")
            obs = env.reset()
```

**Output:**
```
Initial obs: [-0.23  0.45  0.87 -1.12]
Step 0: reward=-1.234, done=False
Step 1: reward=-1.189, done=False
...
Step 47: reward=9.95, done=True
Episode finished!
```

---

## Reward Function Design

Designing good reward functions is **critical** for RL success. Poor rewards lead to:
- Slow learning (sparse rewards)
- Reward hacking (agent exploits loopholes)
- Unsafe behaviors (no safety penalties)

### Principles of Good Reward Functions

1. **Dense Rewards**: Provide feedback at every step (not just terminal states)
2. **Reward Shaping**: Guide agent toward goal with intermediate rewards
3. **Normalized Scale**: Keep rewards in similar magnitude (e.g., all between -10 and +10)
4. **Avoid Reward Hacking**: Prevent exploits (e.g., shaking object instead of grasping)

---

### Example: Manipulation Task

**Task:** Robot arm must grasp an object and place it at a target location.

```python
def compute_reward(state, action):
    """Compute reward for manipulation task"""

    # Extract state components
    ee_pos = state['end_effector_position']
    obj_pos = state['object_position']
    target_pos = state['target_position']
    gripper_closed = state['gripper_closed']
    object_grasped = state['object_grasped']

    # 1. Distance to object (dense reward)
    distance_reward = -np.linalg.norm(ee_pos - obj_pos)

    # 2. Grasping bonus
    if gripper_closed and object_grasped:
        grasp_reward = 5.0
    else:
        grasp_reward = 0.0

    # 3. Success bonus (object at target)
    if np.linalg.norm(obj_pos - target_pos) < 0.05:
        success_reward = 20.0
    else:
        success_reward = 0.0

    # 4. Action penalty (encourage smooth motions)
    action_penalty = -0.01 * np.sum(np.abs(action))

    # 5. Time penalty (encourage fast completion)
    time_penalty = -0.01

    # Total reward
    reward = distance_reward + grasp_reward + success_reward + action_penalty + time_penalty

    return reward
```

**Breakdown:**
- **Distance reward** (-2.0 to 0.0): Guides arm toward object
- **Grasp reward** (+5.0): Encourages closing gripper when touching object
- **Success reward** (+20.0): Final goal completion
- **Action penalty** (-0.02 typical): Discourages jerky motions
- **Time penalty** (-0.01 per step): Encourages efficiency

**Total range:** -2.02 to +22.97 (success episode)

---

### Common Pitfalls

❌ **Sparse Rewards:**
```python
# BAD: Only reward at success
reward = 1.0 if success else 0.0
```
**Problem:** Agent gets no feedback until it randomly stumbles upon success (can take millions of steps).

✅ **Dense Alternative:**
```python
# GOOD: Reward progress toward goal
reward = -distance_to_goal  # Continuous feedback
if success:
    reward += 10.0  # Bonus
```

---

❌ **Reward Hacking:**
```python
# BAD: Reward "object moved"
reward = distance_object_moved
```
**Problem:** Agent learns to shake/throw object (maximizes distance moved) instead of grasping carefully.

✅ **Fix:**
```python
# GOOD: Reward "object moved toward target"
reward = -distance(object, target)
```

---

❌ **Unbalanced Scales:**
```python
# BAD: Action penalty dominates
reward = -1000 * np.sum(action**2) - distance
```
**Problem:** Agent learns to minimize actions (stays still) instead of reaching goal.

✅ **Fix:**
```python
# GOOD: Balance scales
reward = -distance + 10 * success - 0.01 * np.sum(action**2)
```

---

## Training with Stable-Baselines3

**Stable-Baselines3 (SB3)** is a popular RL library with production-ready algorithms.

### Install

```bash
pip install stable-baselines3[extra]
```

### Train PPO Agent

**PPO (Proximal Policy Optimization)** is a good default algorithm for continuous control.

```python
"""
Train PPO agent on Reaching environment.
"""
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

# Create vectorized environment (4 parallel envs for faster training)
vec_env = make_vec_env(lambda: ReachingEnv(), n_envs=4)

# Create PPO agent
model = PPO(
    "MlpPolicy",  # Multi-layer perceptron (feedforward neural network)
    vec_env,
    verbose=1,  # Print training progress
    learning_rate=3e-4,  # Standard for PPO
    n_steps=2048,  # Rollout length per env
    batch_size=64,  # Minibatch size
    n_epochs=10,  # Epochs per rollout
    tensorboard_log="./logs/"  # TensorBoard logging
)

# Train for 100k timesteps
print("Training PPO agent...")
model.learn(total_timesteps=100000)

# Save trained model
model.save("reaching_ppo")
print("Model saved to reaching_ppo.zip")
```

**Expected Output:**
```
---------------------------------
| rollout/           |          |
|    ep_len_mean     | 67.2     |
|    ep_rew_mean     | -8.3     |
| time/              |          |
|    fps             | 2048     |
|    total_timesteps | 10000    |
---------------------------------
...
---------------------------------
| rollout/           |          |
|    ep_len_mean     | 42.1     |
|    ep_rew_mean     | 8.7      |  ← Success!
| time/              |          |
|    total_timesteps | 100000   |
---------------------------------
```

---

### Test Trained Policy

```python
"""
Test trained PPO policy.
"""
from stable_baselines3 import PPO

# Load trained model
model = PPO.load("reaching_ppo")

# Create test environment
env = ReachingEnv()

# Run 10 episodes
for episode in range(10):
    obs = env.reset()
    total_reward = 0
    done = False

    while not done:
        # Get action from policy (deterministic for testing)
        action, _states = model.predict(obs, deterministic=True)

        # Take step
        obs, reward, done, info = env.step(action)
        total_reward += reward

    print(f"Episode {episode + 1}: Total reward = {total_reward:.2f}")
```

**Expected Output:**
```
Episode 1: Total reward = 9.12  ← Success (reward > 0)
Episode 2: Total reward = 8.87
Episode 3: Total reward = 9.45
Episode 4: Total reward = 8.76
...
```

---

### Monitor Training with TensorBoard

```bash
# Start TensorBoard
tensorboard --logdir ./logs/

# Open browser: http://localhost:6006
```

**What to monitor:**
- `rollout/ep_rew_mean`: Average episode reward (should increase)
- `train/loss`: Policy loss (should decrease)
- `train/value_loss`: Value function loss

---

## Sim-to-Real Transfer: Best Practices

To maximize sim-to-real success, apply these strategies during training:

### 1. Domain Randomization

```python
# Randomize physics parameters each episode
def randomize_physics():
    mass = random.uniform(0.8, 1.2) * nominal_mass
    friction = random.uniform(0.5, 1.5) * nominal_friction
    damping = random.uniform(0.9, 1.1) * nominal_damping

    env.set_object_mass(mass)
    env.set_friction(friction)
    env.set_damping(damping)
```

---

### 2. Observation Noise Augmentation

```python
# Add noise to observations during training
def add_observation_noise(obs):
    noise = np.random.normal(0, 0.01, obs.shape)  # 1% noise
    obs_noisy = obs + noise
    return obs_noisy
```

---

### 3. Action Delay Simulation

```python
# Simulate 50ms actuator delay
action_buffer = []

def step_with_delay(action):
    action_buffer.append(action)

    # 50ms delay = 5 steps at 100 Hz physics rate
    if len(action_buffer) > 5:
        delayed_action = action_buffer.pop(0)
    else:
        delayed_action = np.zeros_like(action)  # No action until buffer fills

    # Execute delayed action
    obs, reward, done, info = env.step(delayed_action)
    return obs, reward, done, info
```

---

### 4. Conservative Action Limits

```python
# Limit max velocities during training (transfer with safety margin)
action_clipped = np.clip(action, -0.5, 0.5)  # 50% of max torque
```

---

## Key Takeaways

1. **RL learns policies through trial and error** without explicit programming — the agent discovers strategies from reward signals

2. **MDP framework** (states, actions, rewards, transitions) formalizes decision-making problems

3. **Reward function design is critical** — use dense, shaped rewards to guide learning

4. **Sim-to-real gap is a major challenge** — use domain randomization, observation noise, and action delays to improve transfer

5. **Isaac Gym enables massively parallel training** with 1000x speedup over CPU-based methods (4096 environments on one GPU)

6. **Stable-Baselines3 simplifies RL training** with production-ready algorithms (PPO, SAC, TD3)

7. **PPO is a good default algorithm** for continuous control tasks (stable, sample-efficient)

8. **Successful sim-to-real requires careful environment design** — randomize physics, add noise, simulate delays

---

## Exercises

### Conceptual (2 questions)

**1. Sparse vs Dense Rewards**

Explain why a sparse reward (1.0 for success, 0.0 otherwise) makes learning harder than a dense reward (-distance_to_goal at every step). Provide a concrete example with a reaching task.

<details>
<summary>Hint</summary>

Consider how often the agent receives useful feedback. With sparse rewards, how does the agent know if it's getting closer to the goal or not?
</details>

<details>
<summary>Answer</summary>

**Sparse Reward Problem:**
- Agent only gets feedback (+1.0) when it randomly reaches the goal
- If workspace is 2m x 2m and target is 0.1m radius, probability of random success ≈ 0.00785
- Agent needs ~127 random trials to get one success (on average)
- No gradient: agent doesn't know if moving from 0.5m to 0.4m away is progress

**Dense Reward Solution:**
- `reward = -distance` gives feedback every step
- Moving from 0.5m to 0.4m → reward improves from -0.5 to -0.4
- Agent learns "closer = better" immediately
- Converges in ~10,000 steps vs ~1,000,000 for sparse

**Concrete Example:**
- Episode 1: Start at 1.0m away → Move randomly → End at 0.8m → Dense reward: improved! Sparse reward: 0.0 (no signal)
</details>

---

**2. Domain Randomization for Sim-to-Real**

You trained a grasping policy in Isaac Gym, but when deployed to the real robot, objects slip out of the gripper. What **3 physics parameters** should you randomize during training to improve sim-to-real transfer?

<details>
<summary>Answer</summary>

1. **Friction coefficient** (randomize 0.3-0.9):
   - Real objects have varying friction (plastic vs metal vs wood)
   - Slipping indicates low friction in reality

2. **Object mass** (randomize ±30%):
   - Real objects may be heavier/lighter than CAD models
   - Affects required grip force

3. **Contact stiffness/damping** (randomize ±50%):
   - Affects how gripper "feels" the object
   - Soft objects deform, hard objects don't
   - Influences grasp stability

*Bonus:* Gripper actuator noise (±5% position error) to simulate real servo jitter.
</details>

---

### Coding (3 exercises)

**3. Reward Shaping**

Modify the `ReachingEnv` reward function to include:
1. Bonus for **low joint velocities** (encourage smooth motion)
2. Penalty for **exceeding joint limits** (safety)
3. Bonus for **reaching target quickly** (inverse of timesteps taken)

<details>
<summary>Starter Code</summary>

```python
def step(self, action):
    # ... existing code ...

    # Compute base reward
    reward = -distance

    # TODO: Add velocity bonus
    # Hint: self.joint_angles changed by `action * 0.1`
    # velocity ≈ action magnitude

    # TODO: Add joint limit penalty
    # Hint: Check if self.joint_angles near ±π

    # TODO: Add speed bonus
    # Hint: reward += bonus / (self.timestep + 1)

    return obs, reward, done, {}
```
</details>

---

**4. Custom Environment: Inverted Pendulum**

Create a `BalancingEnv` where a single inverted pendulum (like a Segway) must balance upright.

**Requirements:**
- **State:** `[angle, angular_velocity]`
- **Action:** `[torque]` (continuous, -1 to 1)
- **Reward:** `cos(angle) - 0.01 * angular_velocity^2 - 0.001 * torque^2`
- **Termination:** `|angle| > 30°` or `timestep > 200`

<details>
<summary>Starter Code</summary>

```python
class BalancingEnv(gym.Env):
    def __init__(self):
        self.observation_space = spaces.Box(
            low=np.array([-np.pi, -10]),
            high=np.array([np.pi, 10]),
            dtype=np.float32
        )

        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(1,), dtype=np.float32
        )

        # Pendulum parameters
        self.gravity = 9.81
        self.length = 1.0
        self.dt = 0.02  # 50 Hz

        self.reset()

    def reset(self):
        # Start near upright with small perturbation
        self.angle = np.random.uniform(-0.1, 0.1)
        self.angular_velocity = 0.0
        self.timestep = 0
        return self._get_obs()

    def _get_obs(self):
        return np.array([self.angle, self.angular_velocity], dtype=np.float32)

    def step(self, action):
        torque = action[0]

        # Simple pendulum dynamics (Euler integration)
        angular_accel = (self.gravity / self.length) * np.sin(self.angle) + torque
        self.angular_velocity += angular_accel * self.dt
        self.angle += self.angular_velocity * self.dt

        # Reward: upright (angle=0) + low velocity + low action
        reward = np.cos(self.angle) - 0.01 * self.angular_velocity**2 - 0.001 * torque**2

        # Termination
        done = (abs(self.angle) > np.radians(30)) or (self.timestep > 200)

        self.timestep += 1
        return self._get_obs(), reward, done, {}
```
</details>

---

**5. Train Balancing Policy**

Use Stable-Baselines3 to train a PPO agent on your `BalancingEnv` for 50k timesteps.

**Tasks:**
1. Train the agent
2. Plot the reward curve (use TensorBoard or matplotlib)
3. Test the trained policy for 10 episodes and report success rate

**Success Criteria:** Balance for 200 steps without falling (|angle| < 30°).

<details>
<summary>Starter Code</summary>

```python
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

# Create vectorized environment
vec_env = make_vec_env(lambda: BalancingEnv(), n_envs=4)

# Train PPO
model = PPO("MlpPolicy", vec_env, verbose=1, tensorboard_log="./balance_logs/")
model.learn(total_timesteps=50000)
model.save("balance_ppo")

# Test
model = PPO.load("balance_ppo")
env = BalancingEnv()

successes = 0
for episode in range(10):
    obs = env.reset()
    done = False
    steps = 0

    while not done:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        steps += 1

    if steps == 200:
        successes += 1

print(f"Success rate: {successes}/10")
```
</details>

---

## Further Resources

**RL Foundations:**
- ["Reinforcement Learning: An Introduction"](http://incompleteideas.net/book/the-book.html) (Sutton & Barto) — Free online textbook
- [OpenAI Spinning Up](https://spinningup.openai.com/) — RL tutorial with code

**Isaac Gym:**
- [Isaac Gym Documentation](https://developer.nvidia.com/isaac-gym) — Official guide
- [Isaac Gym Examples](https://github.com/NVIDIA-Omniverse/IsaacGymEnvs) — Pre-built environments

**Stable-Baselines3:**
- [SB3 Documentation](https://stable-baselines3.readthedocs.io/) — Complete API reference
- [SB3 RL Zoo](https://github.com/DLR-RM/rl-baselines3-zoo) — Pre-trained models and hyperparameters

**Research Papers:**
- *Proximal Policy Optimization Algorithms* (Schulman et al., 2017) — PPO paper
- *Sim-to-Real Transfer in Deep Reinforcement Learning for Robotics: a Survey* (2020)
- *Learning Dexterous In-Hand Manipulation* (OpenAI, 2019) — Domain randomization success

**Community:**
- [r/reinforcementlearning](https://www.reddit.com/r/reinforcementlearning/) — Reddit community
- [RL Discord](https://discord.gg/xhfNqQv) — Active discussions

---

**Next:** [Module 4: Vision-Language-Action](../module-4/intro) — Integrate voice commands, LLM planning, and complete autonomous task execution pipelines.
