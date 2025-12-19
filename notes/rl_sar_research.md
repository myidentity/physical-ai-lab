# rl_sar Research Notes

> **Purpose**: Reference document for rl_sar project - deployment of trained RL policies to simulation and real robots.
> **Last Updated**: 2025-12-20
> **Status**: Research complete, ready to test

---

## Overview

**rl_sar** = "Simulation And Real"
- **Author**: Ziqi Fan (fan-ziqi)
- **Repository**: https://github.com/fan-ziqi/rl_sar
- **Stars**: 1k+
- **License**: Apache 2.0

**Purpose**: Take policies trained in Isaac Lab/IsaacGym and deploy them to:
1. Gazebo simulation (ROS-based verification)
2. MuJoCo simulation (lightweight, cross-platform)
3. Real robot hardware (C++ inference)

---

## The Training → Deployment Pipeline

```
┌─────────────────────────────────────────────────────────────────┐
│                    TRAINING (GPU)                                │
│  Isaac Lab + robot_lab → Train policy → Export .pt/.onnx        │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                    VERIFICATION (rl_sar)                         │
│  MuJoCo Simulation → Quick testing without ROS                   │
│  Gazebo Simulation → ROS integration, sensor simulation          │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                    DEPLOYMENT (rl_sar)                           │
│  Real Robot → C++ inference on Jetson/embedded                   │
└─────────────────────────────────────────────────────────────────┘
```

---

## Supported Robots

| Robot | Gazebo | MuJoCo | Real HW | Notes |
|-------|--------|--------|---------|-------|
| **Unitree Go2** | ✅ | ✅ | ✅ | Standard quadruped |
| **Unitree Go2-W** | ✅ | ✅ | ✅ | Wheeled variant (16 DOF) |
| **Unitree G1** | ✅ | ✅ | ✅ | Humanoid (29 DOF) |
| **Unitree A1** | ✅ | ✅ | ✅ | Older quadruped |
| **Unitree B2/B2-W** | ✅ | ✅ | ✅ | Industrial |
| **FFTAI GR1T1/T2** | ✅ | - | ✅ | Ubuntu 20.04 only |
| **Deeprobotics Lite3** | ✅ | ✅ | ✅ | |
| **DDTRobot Tita** | ✅ | ✅ | ✅ | |

---

## Pre-trained Policies Included

rl_sar ships with ready-to-run policies:

| Robot | Policy Config | Training Framework |
|-------|---------------|-------------------|
| Go2 | `himloco` | IsaacGym |
| Go2 | `robot_lab` | IsaacSim/IsaacLab |
| Go2W | `robot_lab` | IsaacSim/IsaacLab |
| A1 | `himloco` | IsaacGym |
| B2/B2W | `robot_lab` | IsaacSim/IsaacLab |

**Policy location**: `rl_sar/src/rl_sar/policy/<ROBOT>/<CONFIG>/policy.pt`

---

## Installation

### Prerequisites

```bash
# Ubuntu 20.04/22.04
sudo apt install cmake build-essential libyaml-cpp-dev libeigen3-dev \
    libboost-all-dev libfmt-dev libtbb-dev liblcm-dev

# For MuJoCo (download from https://mujoco.org/)
# Extract to ~/.mujoco/mujoco-<version>
```

### Clone & Build

```bash
# Clone with submodules
git clone --recursive --depth 1 https://github.com/fan-ziqi/rl_sar.git
cd rl_sar

# Build options:
./build.sh           # Full ROS build (Gazebo)
./build.sh -mj       # CMake + MuJoCo only (RECOMMENDED FOR TESTING)
./build.sh -m        # CMake only (for real robot deployment)
./build.sh -c        # Clean build artifacts
```

---

## Quick Test: MuJoCo Simulation (No ROS)

```bash
# After building with -mj flag:
./cmake_build/bin/rl_sim_mujoco go2w outdoor

# Other scenes: flat, indoor, outdoor, stairs
```

**Controls:**
| Input | Action |
|-------|--------|
| Gamepad Left Stick | Forward/backward, strafe |
| Gamepad Right Stick | Rotate |
| Keyboard WASD | Movement |
| R key | Reset robot |

---

## Gazebo Simulation (ROS Required)

### ROS1 (Noetic)
```bash
# Terminal 1: Launch Gazebo
roslaunch rl_sar gazebo.launch rname:=go2w

# Terminal 2: Run controller
rosrun rl_sar rl_sim
```

### ROS2 (Humble)
```bash
# Terminal 1
ros2 launch rl_sar gazebo.launch.py rname:=go2w

# Terminal 2
ros2 run rl_sar rl_sim
```

---

## Real Robot Deployment

### Network Setup (Go2/Go2W)

1. Connect Ethernet to robot's onboard Jetson
2. Configure computer IP: `192.168.123.x` (e.g., 192.168.123.100)
3. Robot Jetson IP: `192.168.123.161`

### Running on Robot

```bash
# On robot's Jetson (SSH in first)
ssh unitree@192.168.123.161

# Build for hardware
./build.sh -m

# Run
./cmake_build/bin/rl_real_go2 eth0         # Standard Go2
./cmake_build/bin/rl_real_go2 eth0 wheel   # Go2W with wheels
```

---

## Using Your Own Trained Policy

### Export from Isaac Lab

```python
# In Isaac Lab, after training:
# The policy is saved as model_<iteration>.pt in logs folder
# e.g., logs/rsl_rl/unitree_go2w_flat/2025-12-19_17-09-01/model_4995.pt
```

### Copy to rl_sar

```bash
# Create config directory if needed
mkdir -p rl_sar/src/rl_sar/policy/go2w/my_policy/

# Copy policy
cp /path/to/model_4995.pt rl_sar/src/rl_sar/policy/go2w/my_policy/policy.pt

# Copy and modify config.yaml from robot_lab template
cp rl_sar/src/rl_sar/policy/go2w/robot_lab/config.yaml \
   rl_sar/src/rl_sar/policy/go2w/my_policy/config.yaml
```

### Config.yaml Key Parameters

```yaml
policy_path: "policy.pt"
inference_backend: "torch"  # or "onnx"
rl_kp: 20.0                 # PD gains
rl_kd: 0.5
action_scale: 0.25
clip_actions_upper: 1.0
clip_actions_lower: -1.0
```

### Joint Order Compatibility

**CRITICAL**: Joint order in robot_lab `joint_names` must match rl_sar `base.yaml`:

```yaml
# rl_sar/src/rl_sar/policy/go2w/base.yaml
joint_names:
  - FL_hip_joint
  - FL_thigh_joint
  - FL_calf_joint
  - FL_wheel_joint
  - FR_hip_joint
  # ... etc
```

This must match the order in `robot_lab/source/robot_lab/robot_lab/tasks/locomotion/velocity/config/go2w/flat_env_cfg.py`

---

## Directory Structure

```
rl_sar/
├── build.sh                    # Build script
├── cmake_build/                # Build output (after building)
│   └── bin/
│       ├── rl_sim_mujoco       # MuJoCo simulator
│       ├── rl_real_go2         # Real Go2 deployment
│       └── rl_real_go2w        # Real Go2W deployment
├── src/
│   └── rl_sar/
│       ├── policy/             # Pre-trained policies
│       │   ├── go2/
│       │   │   ├── base.yaml   # Robot constants
│       │   │   ├── himloco/    # IsaacGym policy
│       │   │   └── robot_lab/  # IsaacLab policy
│       │   └── go2w/
│       │       ├── base.yaml
│       │       └── robot_lab/
│       ├── models/             # URDF/MJCF models
│       └── scenes/             # MuJoCo scenes
└── README.md
```

---

## Relationship to robot_lab

| Project | Purpose | Where |
|---------|---------|-------|
| **robot_lab** | Training environments for Isaac Lab | GPU training machine |
| **rl_sar** | Deployment to sim/real | Any machine / robot |

**Workflow:**
1. Train in Isaac Lab using robot_lab environments
2. Export policy (.pt file)
3. Deploy with rl_sar to MuJoCo/Gazebo/Real robot

---

## Author: Ziqi Fan (fan-ziqi)

- **GitHub**: https://github.com/fan-ziqi
- **Email**: fanziqi614@gmail.com
- **Website**: http://www.robotsfan.com
- **Education**: ZSTU (Robotics), NJUST (Mechanical Engineering)
- **Location**: Nanjing, China

---

## TODO: Testing Plan

- [ ] Clone rl_sar repository
- [ ] Install MuJoCo dependencies
- [ ] Build with `./build.sh -mj`
- [ ] Test pre-trained Go2W policy in MuJoCo
- [ ] Document the experience
- [ ] Try with our own trained policy from Isaac Lab
- [ ] Add to blog post if successful

---

## References

- [rl_sar GitHub](https://github.com/fan-ziqi/rl_sar)
- [robot_lab GitHub](https://github.com/fan-ziqi/robot_lab)
- [DeepWiki Go2 Implementation](https://deepwiki.com/fan-ziqi/rl_sar/4.2-unitree-go2-implementation)
- [rl_sar Releases](https://github.com/fan-ziqi/rl_sar/releases)
