# Plan: ROSCon India 2025 Workshop Infrastructure Blog Post (UPDATED)

## Overview

Create a comprehensive blog post documenting the Docker infrastructure built for ROSCon India 2025 workshops. This post serves three purposes:

1. **Help Others**: Guide for workshop organizers at future conferences
2. **Help Ourselves**: Reference for future workshop setups
3. **Claude Reference**: Enable Claude to assist with future workshop infrastructure using this as context

**Key Update**: This plan reflects Phase 7.5 modernization - switching from deprecated TurtleBot3 (Gazebo Classic EOL Jan 2025) to modern ros-gz-sim-demos.

---

## Blog Framework: Quarto

- **Location**: `${HOME}/physical-ai-lab-blog/blog/posts/`
- **Format**: `.qmd` (Quarto Markdown)
- **Structure**: `YYYY-MM-DD-post-slug/index.qmd`

---

## Post Details

### File Path
```
${HOME}/physical-ai-lab-blog/blog/posts/2025-12-15-roscon-india-workshop-docker-infrastructure/
├── index.qmd              (main post)
├── thumbnail.png          (small hero image, <500KB)
├── screenshots/           (terminal/GUI screenshots)
│   ├── launch-menu.png
│   ├── gazebo-sim-diff-drive.png    (NEW: ros-gz-sim-demos)
│   ├── rviz2-demo.png
│   └── bridge-test.png
└── code/                  (sanitized, reusable snippets)
    ├── cyclonedds.xml
    ├── docker-compose-template.yml
    ├── Dockerfile-snippet-isaac-ros.dockerfile
    ├── Dockerfile-snippet-jazzy.dockerfile
    ├── Makefile-key-targets.md
    ├── offline-save-snippet.sh
    ├── launch-container-snippets.sh       (NEW: key functions from 500-line script)
    └── new-terminal.sh                    (NEW: full 87-line script - it's small enough)
```

### What Goes in Blog Git vs Private Repo

| Include in Blog Git | Keep in Private Repo Only |
|---------------------|---------------------------|
| ✅ Sanitized config templates | ❌ Full Dockerfiles (use key snippets) |
| ✅ Small screenshots (<1MB each) | ❌ Large build logs |
| ✅ Code snippets (50-150 lines max) | ❌ Full 600-line scripts |
| ✅ ASCII diagrams (inline) | ❌ Complete workspace files |
| ✅ CycloneDDS XML (safe) | ❌ Credentials/API keys |
| ✅ TURTLEBOT3_MIGRATION.md (as example) | ❌ Full git history |
| ✅ Troubleshooting patterns | ❌ Internal notes/TODOs |

**Sharing Strategy**: Copy sanitized versions of key files to blog repo. Link to blog repo in post, not private repo.

---

## Frontmatter (UPDATED)

```yaml
---
title: "Building a Modern Offline ROS 2 Workshop Infrastructure: ROSCon India 2025"
description: "How we built Docker infrastructure with modern Gazebo Sim, GPU acceleration, DDS↔Zenoh bridging, complete offline support, and handled last-minute deprecation challenges"
author: "Physical AI Lab Team"
date: "2025-12-15"
categories: [ROS2, Docker, Zenoh, Workshop, ROSCon, NVIDIA, Gazebo]
image: "thumbnail.png"
draft: false
format:
  html:
    code-fold: false
    code-tools: true
    toc: true
    toc-depth: 3
---
```

---

## Post Outline (UPDATED)

### 1. TL;DR Section
**What we built:**
- 7 Docker images for offline ROS 2 workshops (Dec 18-20, COEP Pune)
- Modern Gazebo Sim (NOT Classic - avoided EOL trap!)
- GPU acceleration, Zenoh bridging, Nav2 complete
- 100% offline-capable with bundled models
- Multi-role certification process for quality assurance

**Key innovation:**
- Discovered TurtleBot3 uses deprecated Gazebo Classic (EOL Jan 2025)
- Pivoted to ros-gz-sim-demos before committing to 80GB offline tars
- Saved workshop from teaching deprecated technology

**Who it's for:**
- Workshop organizers at robotics conferences
- ROS 2 developers setting up training environments
- Anyone building reproducible robotics education infrastructure

### 2. The Problem

**Context:**
- ROSCon India 2025: First ROSCon in India (Dec 18-20, COEP Pune)
- Two workshops: Zenoh Networking (Workshop 3) + IMU Perception (Workshop 4)
- ~50-100 participants with diverse hardware

**Challenges:**
1. **Unreliable Internet**: Venue WiFi can't handle 100 concurrent apt updates
2. **GPU Requirements**: VSLAM, NVBlox need NVIDIA acceleration
3. **Middleware Diversity**: Need to demo CycloneDDS AND Zenoh
4. **Real Robot Integration**: Unitree Go2 quadruped with proprietary SDK
5. **Version Hell**: Participants on Ubuntu 20.04, 22.04, 24.04 mix
6. **Late Discovery**: TurtleBot3 uses EOL Gazebo Classic (found at Phase 7!)

**Decision Point:**
Pause Phase 8 (offline prep) to modernize rather than commit to 80GB of deprecated software.

### 3. The Journey: From TurtleBot3 to ros-gz-sim-demos

**Initial Plan (Phase 1-6):**
- Use TurtleBot3 (familiar, lots of tutorials)
- Gazebo Classic simulation
- Standard approach everyone uses

**Phase 7 Discovery:**
```bash
# Testing TurtleBot3
ros2 launch turtlebot3_gazebo empty_world.launch.py
# [WARN] Gazebo Classic is end-of-life. Please migrate to new Gazebo.
```

**Research:**
- Gazebo Classic EOL: January 31, 2025 (6 weeks after workshop!)
- Alternative: ros-gz-sim-demos uses modern Gazebo Sim
- Trade-off: Less familiar but future-proof

**The Pivot (Phase 7.5):**
- Removed all TurtleBot3 packages
- Added ros-gz-sim-demos (19 official demos)
- Completed Nav2 stack installation
- Created migration guide for participants
- Re-tested everything offline

**Why This Matters:**
Teaching deprecated technology would have required rework in 6 months. Pausing at Phase 7 saved weeks of future work.

### 4. Architecture Overview

**Three-Tier Docker Image Strategy:**

```
TIER 1: Base Images (pulled from registries)
├── nvcr.io/nvidia/isaac/ros:x86_64-ros2_humble (~22GB)
└── osrf/ros:jazzy-desktop-full (~3.5GB)

TIER 2: Custom Base Images (build locally)
├── isaac-ros-base:humble (VSLAM, NVBlox, RealSense, Go2 SDK, Nav2, ros-gz-sim-demos)
└── jazzy-base:latest (MuJoCo, Claude Code, Playwright, Nav2, ros-gz-sim-demos)

TIER 3: Workshop Images (docker-compose)
├── workshop3-humble-dds (CycloneDDS + Zenoh Bridge)
├── workshop3-jazzy-zenoh (rmw_zenoh via apt)
├── workshop3-humble-zenoh (rmw_zenoh source - BROKEN, needs Rust)
├── workshop4-imu (IMU tools, robot_localization)
└── robot-humble (Jetson communication)
```

**Two-Track Rationale:**

**Track A: NVIDIA Isaac ROS (Humble)**
- GPU-accelerated perception (VSLAM, NVBlox)
- CycloneDDS 0.10.2 (required by Unitree Go2 SDK)
- Mature ecosystem for robotics

**Track B: OSRF Jazzy**
- Latest ROS 2 LTS
- rmw_zenoh available via apt (no source build needed)
- Cleaner for Zenoh workshop demos

**Why Not One Image?**
- NVIDIA base image is 22GB (overkill for Zenoh demos)
- Different RMW implementations (CycloneDDS vs Zenoh)
- Separation of concerns (GPU perception vs networking)

### 5. The Docker Images (7 total)

| Image | Base | Size | Purpose | Status |
|-------|------|------|---------|--------|
| **isaac-ros-base** | NVIDIA Isaac ROS Humble | 24.1GB | GPU perception, Go2 SDK, Nav2, ros-gz-sim-demos | ✅ Production |
| **jazzy-base** | OSRF Jazzy | 2.85GB | Latest ROS 2, rmw_zenoh, ros-gz-sim-demos | ✅ Production |
| **workshop3-humble-dds** | isaac-ros-base | 24.2GB | CycloneDDS + zenoh-bridge v1.7.1 | ✅ Primary W3 |
| **workshop3-jazzy-zenoh** | jazzy-base | 2.95GB | Native rmw_zenoh v0.2.9 | ✅ Primary W3 (alt) |
| **workshop3-humble-zenoh** | isaac-ros-base | 24.2GB | rmw_zenoh source (needs Rust) | ❌ Broken (known) |
| **workshop4-imu** | isaac-ros-base | 24.2GB | imu_tools, robot_localization, rtabmap | ✅ Production |
| **robot-humble** | isaac-ros-base | 24.1GB | Same as base (Jetson comms) | ✅ Production |

**Total Disk**: ~150GB uncompressed → ~80-95GB compressed (zstd -19)

### 6. Key Technical Innovations

#### 6.1 Modern Gazebo Migration

**The Problem:**
```bash
# OLD (deprecated)
ros2 launch turtlebot3_gazebo empty_world.launch.py
# Uses Gazebo Classic (EOL Jan 2025)
```

**The Solution:**
```bash
# NEW (modern)
ros2 launch ros_gz_sim_demos diff_drive.launch.py
# Uses Gazebo Sim (Ignition Fortress in Humble, Gazebo Sim in Jazzy)
```

**Benefits:**
- 19 official demos vs 3 TurtleBot3 worlds
- Two robots spawned by default (blue + green vehicles)
- Active maintenance by Open Robotics
- Future-proof for years

**Discovery: Models Are Bundled!**
```bash
# Models embedded in Debian package (no Fuel download needed!)
/opt/ros/jazzy/share/ros_gz_sim_demos/models/vehicle/
/opt/ros/jazzy/share/ros_gz_sim_demos/worlds/vehicle.sdf
```

This eliminates internet dependency for workshop demos.

#### 6.2 Shared Gazebo Fuel Cache Pattern

**Initial Plan:** Bake models into Docker images
```dockerfile
# Seemed logical but wasteful
RUN gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Depot"
```

**Problem:** 4 containers × 50MB models = 200MB duplicates

**Innovation:** Shared volume caches
```yaml
# docker-compose.yml
volumes:
  - ./cache/ignition:/root/.ignition:rw  # Humble (ign command)
  - ./cache/gz:/root/.gz:rw              # Jazzy (gz command)
```

**Critical Discovery:** Humble uses `ign`, Jazzy uses `gz`
```bash
# Humble (Ignition Fortress - pre-rebranding)
ign fuel download -u "https://..."
# Cache: ~/.ignition/fuel/

# Jazzy (Gazebo Sim - post-rebranding)
gz fuel download --url "https://..."
# Cache: ~/.gz/fuel/
```

**Benefits:**
- Download once, share everywhere
- Disk usage: 200MB → 50MB (75% reduction)
- Build time: 32 min → 15 min (53% faster)
- Consistency across containers

**For Workshop:** Pre-populate cache before event, participants load from cache

#### 6.3 DDS ↔ Zenoh Bridging Architecture

**Use Case:** Robot (Jetson) with CycloneDDS → Laptop with rmw_zenoh

```
Robot (Jetson Orin)              WiFi              Laptop
┌──────────────┐         ┌──────────────┐         ┌──────────────┐
│ CycloneDDS   │◄───────►│ zenoh-bridge │◄───────►│ rmw_zenoh    │
│ (can't change│  Domain │ -ros2dds     │  Zenoh  │ (efficient)  │
│  middleware) │   99    │              │  Router │              │
└──────────────┘         └──────────────┘         └──────────────┘
```

**Why Bridge Instead of Native Zenoh?**
- Robot firmware uses CycloneDDS (can't change)
- Zenoh more efficient over WiFi (bandwidth savings)
- Bridge allows gradual migration

**Configuration:**
```bash
# Terminal 1: Router mode (laptop)
zenoh-bridge-ros2dds -m router -d 99

# Terminal 2: Peer mode (another laptop/node)
zenoh-bridge-ros2dds -m peer -d 88
```

**Tested Results:**
- 15 consecutive messages bridged successfully
- Auto-discovered topics: `/chatter`, `/rosout`, `/parameter_events`
- Latency: <10ms (local network)

#### 6.4 GPU Sharing Across Containers

**Hardware:** NVIDIA RTX 5090 (24GB VRAM)

**Challenge:** Multiple containers need GPU simultaneously

**Solution:**
```yaml
# docker-compose.yml
services:
  workshop3-dds:
    runtime: nvidia
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
```

**Verified:**
```bash
# Inside container
nvidia-smi  # Shows RTX 5090
glxinfo | grep "direct rendering"  # Yes
```

**Use Cases:**
- VSLAM (Isaac ROS)
- NVBlox (real-time mapping)
- Gazebo Sim rendering
- RViz2 visualization

#### 6.5 Complete Nav2 Stack

**Installed Packages (per container):**
- Humble: 30-31 Nav2 packages
- Jazzy: 34 Nav2 packages

**Verified Nodes:**
- `nav2_amcl` - Localization ✅
- `nav2_planner` - Path planning ✅
- `nav2_controller` - Path following ✅
- `nav2_bt_navigator` - Behavior trees ✅
- `nav2_map_server` - Map loading ✅
- `nav2_lifecycle_manager` - Node orchestration ✅

**Testing:**
```bash
# Launch individual nodes (verified working)
ros2 run nav2_amcl amcl
ros2 run nav2_planner planner_server
```

**Status:** Production-ready for autonomous navigation workshops

#### 6.6 Offline-First Design

**Challenge:** Workshop has no reliable internet

**Strategy:**

1. **Pre-install Everything:**
   - All ROS 2 packages via apt
   - Python deps via pip (cached)
   - Gazebo models bundled in ros-gz-sim-demos package

2. **Shared Caches:**
   ```yaml
   volumes:
     - ./cache/pip:/root/.cache/pip
     - ./cache/colcon:/root/.colcon
     - ./cache/ignition:/root/.ignition
     - ./cache/gz:/root/.gz
   ```

3. **Docker Image Saves:**
   ```bash
   # Save images to tar files
   docker save isaac-ros-base:humble | zstd -T0 -19 > offline/isaac-ros-base.tar.zst
   # 24.1GB → ~12-15GB compressed
   ```

4. **Verification:**
   ```bash
   # Test with network disabled
   docker run --network none jazzy-base:latest bash -c \
     "ros2 launch ros_gz_sim_demos diff_drive.launch.py"
   # ✅ Works perfectly!
   ```

**Offline Test Results:**
- All demos launch without internet
- Models load from bundled package
- No Fuel downloads attempted
- Cache miss handled gracefully

#### 6.7 RealSense D435i + VSLAM Deployment: Debugging Three Hidden Issues

**Context:** Phase 7.6 tested RealSense D435i camera with Isaac ROS Visual SLAM across all containers. What seemed like straightforward hardware testing turned into a debugging masterclass.

**The Journey:**

**Issue 1: IMU Access Denied (workshop4-imu container)**

**Symptom:**
```bash
# Inside container
rs-enumerate-devices

# Error:
Device or resource busy
```

**Debugging:**
- RealSense D435i has IMU (gyro + accelerometer) via HID interface
- Docker needed permission to access HID devices
- Standard USB passthrough wasn't enough

**Solution:**
```yaml
# docker-compose.yml - workshop4-imu service
device_cgroup_rules:
  - 'c 13:* rmw'  # Character device class 13 (HID input devices)
  - 'c 189:* rmw' # USB device class 189
```

**Why This Worked:**
- `device_cgroup_rules` grants container access to device classes
- Class 13 = HID (Human Interface Devices) - includes IMU
- Class 189 = USB devices
- `rmw` = read, mknod, write permissions

**Issue 2: VSLAM Namespace Collision**

**Symptom:**
```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py

# Camera topics published at:
/camera/infra1/image_rect_raw
/camera/infra2/image_rect_raw

# But VSLAM looked for:
/camera/camera/infra1/image_rect_raw  ❌ (namespace doubled!)
```

**Root Cause Analysis:**
```python
# isaac_ros_visual_slam_realsense.launch.py (original)
Node(
    package='isaac_ros_visual_slam',
    executable='isaac_ros_visual_slam',
    namespace='camera',  # ← Added this namespace
    parameters=[...]
)

# Combined with realsense2_camera default namespace='camera'
# Result: /camera/camera/* ❌
```

**Solution Strategy:**
- Could modify launch file in package → breaks on updates
- Could rebuild package → time-consuming
- **Best:** Docker volume mount override pattern

```yaml
# docker-compose.yml
volumes:
  # Override launch file with fixed version
  - ./fixes/isaac_ros_visual_slam_realsense.launch.py:/opt/ros/humble/share/isaac_ros_visual_slam/launch/isaac_ros_visual_slam_realsense.launch.py:ro
```

**Fixed Launch File:**
```python
# fixes/isaac_ros_visual_slam_realsense.launch.py
Node(
    package='isaac_ros_visual_slam',
    executable='isaac_ros_visual_slam',
    namespace='',  # ← Changed to empty string
    parameters=[
        # Topics now match RealSense defaults
        {'visual_slam/image_0': '/camera/infra1/image_rect_raw'},
        {'visual_slam/image_1': '/camera/infra2/image_rect_raw'},
        ...
    ]
)
```

**Issue 3: ROS_LOCALHOST_ONLY Discovery Mystery**

**Symptom:**
```bash
# workshop3-dds container: ✅ VSLAM works perfectly
# workshop4-imu container: ❌ No camera topics appear
# robot-humble container: ❌ No camera topics appear

# Yet all three containers use the SAME isaac-ros-base image!
```

**Debugging Journey:**

*Hypothesis 1: Camera in use by another container?*
```bash
# Stopped ALL containers, relaunched workshop4-imu alone
# Result: Still failed ❌
```

*Hypothesis 2: Container corruption during build?*
```bash
# Rebuilt workshop4-imu and robot-humble from scratch
docker compose build --no-cache workshop4-imu robot-humble
# Result: Still failed ❌
```

*Hypothesis 3: Environment variable differences?*
```bash
# Compared docker-compose.yml environment sections
# Found the difference!

# workshop3-dds Dockerfile:
ENV ROS_LOCALHOST_ONLY=1  ✅

# workshop4-imu Dockerfile:
ENV ROS_LOCALHOST_ONLY=0  ❌

# robot-humble Dockerfile:
ENV ROS_LOCALHOST_ONLY=0  ❌
```

**Root Cause:**
- `ROS_LOCALHOST_ONLY=1` enables localhost-only DDS discovery
- Prevents network-based discovery conflicts
- Critical for containers with multiple RMW implementations
- workshop3-dds worked by accident (had it in Dockerfile)
- workshop4-imu and robot-humble inherited base image value (0)

**Solution:**
```yaml
# docker-compose.yml - Add to workshop4-imu and robot-humble
environment:
  - ROS_LOCALHOST_ONLY=1  # Use localhost-only DDS (required for topic discovery)
```

**Why This Matters:**
- Same base image, different behavior = environment variable issue
- Debugging required systematic elimination (not just rebuilding)
- Documentation prevents future confusion

**Testing Results:**

| Container | D435i Basic | VSLAM Odometry | Map Saving | IMU Data |
|-----------|-------------|----------------|------------|----------|
| workshop3-dds | ✅ PASS | ✅ PASS | ✅ PASS (2.3MB nvmap) | N/A |
| workshop4-imu | ✅ PASS | ✅ PASS | ✅ PASS (2.1MB nvmap) | ✅ PASS (~9.8 m/s²) |
| robot-humble | ✅ PASS | ✅ PASS | ✅ PASS (2.2MB nvmap) | N/A |

**Lessons from VSLAM Debugging:**

1. **Layer Your Testing**: Basic (camera) → Advanced (VSLAM) → Expert (map saving)
2. **Document Hypotheses**: Write down what you're testing and why
3. **Environment Variables Matter**: Check ALL env vars, not just obvious ones
4. **Volume Mount Overrides**: Powerful pattern for fixing upstream issues
5. **HID Devices Need Special Permissions**: USB passthrough ≠ HID access

**Blog Value:**
- Shows REAL debugging process (not just "it works!")
- Three distinct issues, three different solution patterns
- Teaches systematic troubleshooting methodology
- Saves readers from repeating our mistakes

#### 6.8 RTX 5090 Hardware Verification: When Newer Isn't Supported (Yet It Works)

**Context:** Phase 7.7 tested NVIDIA RTX 5090 (Blackwell architecture, Compute Capability 12.0) with our Docker images. The GPU was so new, official documentation didn't list it.

**The Challenge:**

**Hardware:**
- RTX 5090 (released Dec 2024, Blackwell architecture)
- Compute Capability 12.0 (newest available)
- CUDA 12.6+ required officially

**Installed Software:**
- Host driver: 570.86.16 (supports CUDA 12.8)
- Isaac ROS base image: Built for CUDA 12.2
- Jazzy base image: No CUDA toolchain
- PyTorch containers: Unknown CUDA version support

**The Question:**
> Will Compute Capability 12.0 work with CUDA 12.2 containers via driver forward compatibility?

**Research Phase:**

**Official NVIDIA Documentation:**
```
Supported GPUs (CUDA 12.2):
- RTX 40 series (Ada Lovelace, Compute 8.9)
- RTX 30 series (Ampere, Compute 8.6)
- ...
[RTX 5090 NOT listed] ❌
```

**Driver Forward Compatibility Hypothesis:**
- NVIDIA drivers support NEWER CUDA toolkits than installed
- **But can they support NEWER GPU architectures?**
- Blackwell (Compute 12.0) with CUDA 12.2 containers?

**The Test:**

**Test 1: Basic GPU Detection (isaac-ros-base)**
```bash
docker compose run --rm workshop3-dds bash

# Inside container:
nvidia-smi

# Output:
+-----------------------------------------------------------------------------------------+
| NVIDIA-SMI 570.86.16              Driver Version: 570.86.16      CUDA Version: 12.8     |
|-------------------------------+----------------------+----------------------+
|   0  NVIDIA GeForce RTX 5090   | 00000000:01:00.0 Off |                  Off |
```
✅ **PASS** - GPU detected, driver version shown

**Test 2: CUDA Toolkit Compatibility Check**
```bash
# Check what CUDA version the container thinks it has
nvcc --version

# Output:
Cuda compilation tools, release 12.2, V12.2.140
```

**Test 3: GPU Compute Capability Query**
```bash
# Python test script
python3 << EOF
import torch
print(f"PyTorch CUDA available: {torch.cuda.is_available()}")
print(f"GPU name: {torch.cuda.get_device_name(0)}")
print(f"Compute capability: {torch.cuda.get_device_capability(0)}")
EOF

# Output:
PyTorch CUDA available: True
GPU name: NVIDIA GeForce RTX 5090
Compute capability: (12, 0)  ← Blackwell architecture confirmed!
```
✅ **PASS** - PyTorch recognizes GPU, Compute 12.0 detected

**Test 4: Real Workload (PyTorch CUDA 12.8 Container)**

**Challenge:**
- Isaac ROS base has CUDA 12.2
- Need to test with CUDA 12.8 (matches driver)
- PyTorch official image: `pytorch/pytorch:2.7.0-cuda12.8-cudnn9-runtime`

**The Download Drama:**

```bash
# Started docker pull in background
docker pull pytorch/pytorch:2.7.0-cuda12.8-cudnn9-runtime &

# 1 hour 20 minutes later... is it stuck?
# Created bandwidth monitoring script
```

**Network Bandwidth Monitoring Discovery:**

```bash
#!/bin/bash
# /tmp/check_bandwidth.sh - Created to debug "stuck" download

IFACE=$(ip route get 8.8.8.8 | grep -oP 'dev \K\S+' | head -1)
echo "Monitoring interface: $IFACE"

RX1=$(cat /proc/net/dev | grep "$IFACE" | awk '{print $2}')
sleep 5
RX2=$(cat /proc/net/dev | grep "$IFACE" | awk '{print $2}')

DIFF=$((RX2 - RX1))
SPEED_MB_PER_SEC=$((DIFF / 5 / 1024 / 1024))

echo "Download Speed: $SPEED_MB_PER_SEC MB/s"
```

**Result:**
```
Download Speed: 23 MB/s ✅
```

**Not stuck!** Just a large image (12.2GB). Patience required.

**The PyTorch Test:**
```bash
# After download completed
docker run --rm --gpus all \
  pytorch/pytorch:2.7.0-cuda12.8-cudnn9-runtime \
  python -c "
import torch
print(f'PyTorch version: {torch.__version__}')
print(f'CUDA available: {torch.cuda.is_available()}')
print(f'CUDA version: {torch.version.cuda}')
print(f'GPU count: {torch.cuda.device_count()}')
print(f'GPU name: {torch.cuda.get_device_name(0)}')
print(f'Compute capability: {torch.cuda.get_device_capability(0)}')

# Create tensor on GPU (real workload)
x = torch.randn(1000, 1000, device='cuda')
y = torch.matmul(x, x.T)
print(f'GPU computation: ✅ {y.shape}')
"

# Output:
PyTorch version: 2.7.0
CUDA available: True
CUDA version: 12.8
GPU count: 1
GPU name: NVIDIA GeForce RTX 5090
Compute capability: (12, 0)
GPU computation: ✅ torch.Size([1000, 1000])
```

✅ **PASS** - RTX 5090 Compute 12.0 works with CUDA 12.8 container!

**The Verdict:**

**Driver Forward Compatibility DOES Support Newer GPUs!**
- Host driver 570.86.16 supports CUDA 12.8
- CUDA 12.2 containers work via driver compatibility
- CUDA 12.8 containers work natively
- Compute Capability 12.0 (Blackwell) fully supported
- No code changes needed

**What We Learned:**

1. **Driver Version > Container CUDA Version**
   - Driver 570.86.16 provides CUDA 12.8 runtime
   - Containers with older CUDA (12.2) use driver's newer runtime
   - "Forward compatibility" applies to GPU architecture too

2. **Network Patience Debugging**
   - Created `/proc/net/dev` monitoring script
   - Learned to check bandwidth before assuming "stuck"
   - 12.2GB downloads take time even at 23 MB/s (~9 minutes)

3. **Bleeding-Edge Hardware Testing**
   - RTX 5090 not in official docs yet
   - But Blackwell architecture supported
   - Test, don't assume based on docs alone

**Blog Value:**
- Shows hardware compatibility verification process
- Network debugging tool (bandwidth monitoring)
- Proves forward compatibility with newest GPU
- Saves readers from "will it work?" uncertainty

**Updated Compatibility Matrix:**

| Component | Installed | Tested With | Status |
|-----------|-----------|-------------|--------|
| Host Driver | 570.86.16 | - | ✅ CUDA 12.8 runtime |
| Isaac ROS Base | CUDA 12.2 | RTX 5090 (Compute 12.0) | ✅ Works via driver |
| PyTorch Container | CUDA 12.8 | RTX 5090 (Compute 12.0) | ✅ Native support |
| Workshop Containers | CUDA 12.2 | RTX 5090 (Compute 12.0) | ✅ All GPU features work |

**Conclusion:** RTX 5090 fully compatible with our infrastructure. Workshop demos will have GPU acceleration!

### 7. Build & Test Automation

**Makefile Targets:**

```makefile
make all           # Build everything (~60-90 min first time)
make base          # Build TIER 2 base images (~50 min)
make workshop3     # Build all 3 workshop3 variants (~5 min)
make test          # Smoke tests (ROS2, Gazebo, Zenoh)
make offline-save  # Create compressed tars (~2 hours, 80-95GB)
make offline-load  # Load from tars
make status        # Show built images and sizes
```

**Build Features:**

1. **Timestamp + Git Hash Tagging:**
   ```bash
   IMAGE_TAG := 20251214-103538-a177e48
   # Enables rollback to any build
   ```

2. **Parallel Builds:**
   ```makefile
   # Build bases in parallel (if dependencies allow)
   make base-isaac & make base-jazzy
   ```

3. **Smoke Tests:**
   ```bash
   make test-dds      # CycloneDDS + zenoh-bridge
   make test-jazzy    # rmw_zenoh
   make test-imu      # IMU packages
   make test-bridge   # DDS↔Zenoh bridging
   ```

**Build Times (with cache):**
- isaac-ros-base: ~2 min (first: ~35-45 min)
- jazzy-base: ~5 min (first: ~10-15 min)
- workshop3-*: ~20 sec each
- Total rebuild: ~7 min (vs ~60 min fresh)

**Cache Strategy:**
- Docker BuildKit enabled
- Layer caching maximized
- Separate apt update layers
- Conditional COPY for configs

### 8. Configuration Deep Dive

#### 8.1 CycloneDDS Configuration

**File:** `configs/cyclonedds.xml`

**Key Settings:**
```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain id="any">
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>false</AllowMulticast>  <!-- WiFi doesn't multicast -->
    </General>
    <Discovery>
      <EnableTopicDiscovery>true</EnableTopicDiscovery>
    </Discovery>
  </Domain>
</CycloneDDS>
```

**Why Disable Multicast?**
- WiFi APs often block multicast packets
- Unicast discovery more reliable
- Workshop venue WiFi tested without multicast

**Usage:**
```bash
export CYCLONEDDS_URI=file:///config/cyclonedds.xml
```

#### 8.2 Zenoh Router Configuration

**File:** `configs/zenoh/config.json5`

**Key Settings:**
```json5
{
  mode: "router",
  connect: {
    endpoints: ["tcp/0.0.0.0:7447"]
  },
  scouting: {
    multicast: {
      enabled: false  // Same reason as CycloneDDS
    }
  }
}
```

**Router vs Peer Modes:**
- **Router**: Central hub (1 per network)
- **Peer**: Direct connections (N nodes)

**Workshop Setup:**
- 1 router (instructor laptop)
- N peers (participant laptops)

#### 8.3 Docker Compose Orchestration

**Key Patterns:**

1. **Service Definition Template:**
   ```yaml
   services:
     workshop3-dds:
       build:
         context: ./images/workshop3-humble-dds
         dockerfile: Dockerfile
       image: workshop3-humble-dds:latest
       container_name: workshop3-dds
       hostname: workshop3-dds
       runtime: nvidia
       network_mode: host
       privileged: true
       environment:
         - DISPLAY=${DISPLAY}
         - ROS_DOMAIN_ID=10
         - CYCLONEDDS_URI=file:///config/cyclonedds.xml
       volumes:
         - /tmp/.X11-unix:/tmp/.X11-unix:rw
         - ./workspaces/isaac_ros-dev:/workspaces/isaac_ros-dev:rw
         - ./configs/cyclonedds.xml:/config/cyclonedds.xml:ro
         - ./cache/ignition:/root/.ignition:rw
   ```

2. **Volume Mounting Strategy:**
   - **Workspaces**: `:rw` (read-write, persisted)
   - **Configs**: `:ro` (read-only, immutable)
   - **Caches**: `:rw` (shared, mutable)
   - **X11**: `:rw` (GUI support)

3. **Network Modes:**
   - `host`: ROS 2 discovery (recommended)
   - `bridge`: Isolated (for testing)

### 9. User Experience Innovation: Making Docker Approachable

**The Challenge:** Workshop participants range from Docker experts to complete beginners. Running containers with 15+ flags is intimidating.

**Our Solution:** Two complementary scripts that transform the workshop experience.

---

#### 9.1 Interactive Launch Script: Professional Polish

**File:** `scripts/launch-container.sh` (500 lines)

**The Problem:**
```bash
# What beginners would need to type (actual docker run command):
docker run --rm -it --name roscon-workshop3-dds --hostname workshop3-dds \
  --runtime nvidia --gpus all --network host --privileged \
  -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///config/cyclonedds.xml \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/.Xauthority:/root/.Xauthority:rw \
  -v ./workspaces:/workspaces:rw -v ./configs:/config:ro \
  workshop3-humble-dds:latest bash
# Overwhelming! 😰
```

**What They Actually Type:**
```bash
./launch-container.sh 1
# That's it! 😊
```

---

**Script Features (Why It's Blog-Worthy):**

**1. Professional ASCII Art Banner**
```
╔═══════════════════════════════════════════════════════════════════════════════╗
║                                                                               ║
║   ██████╗  ██████╗ ███████╗ ██████╗ ██████╗ ███╗   ██╗    ██╗███╗   ██╗      ║
║   ██╔══██╗██╔═══██╗██╔════╝██╔════╝██╔═══██╗████╗  ██║    ██║████╗  ██║      ║
║   ██████╔╝██║   ██║███████╗██║     ██║   ██║██╔██╗ ██║    ██║██╔██╗ ██║      ║
║   ██╔══██╗██║   ██║╚════██║██║     ██║   ██║██║╚██╗██║    ██║██║╚██╗██║      ║
║   ██║  ██║╚██████╔╝███████║╚██████╗╚██████╔╝██║ ╚████║    ██║██║ ╚████║      ║
║   ╚═╝  ╚═╝ ╚═════╝ ╚══════╝ ╚═════╝ ╚═════╝ ╚═╝  ╚═══╝    ╚═╝╚═╝  ╚═══╝      ║
║                                                                               ║
║                    India 2025 Workshop - Container Launcher                   ║
║                         December 18-20, COEP Pune                             ║
╚═══════════════════════════════════════════════════════════════════════════════╝
```
*Sets professional tone immediately*

**2. Color-Coded Status Indicators**
```
Select a container to launch:

  1) ● workshop3-dds       [humble]     CycloneDDS + Zenoh Bridge
     └─ workshop3-humble-dds:latest (24.2GB)

  2) ● workshop3-jazzy     [jazzy]      ROS 2 Jazzy + rmw_zenoh
     └─ workshop3-jazzy-zenoh:latest (2.95GB)

  3) ○ workshop3-humble    [humble]     ROS 2 Humble + rmw_zenoh (partial)
     └─ workshop3-humble-zenoh:latest (not built)
```
- **Green ●** = Image built and ready
- **Red ○** = Image not built
- **Shows image size** for disk planning

**3. Pre-flight Checks (Lines 116-188)**
```bash
System Checks
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ✓  Docker 29.1.2
  ✓  NVIDIA Container Toolkit
  ✓  Display: :0
  ✓  GPU: NVIDIA GeForce RTX 5090
  ✓  X11 access granted

Credentials:
  ✓  Git config: Rajesh Kumar
  ✓  Git credentials: ~/.git-credentials
  ✓  Claude: Credentials found in shared/.claude (persists across containers)
```
*Catches issues BEFORE launching container*

**4. Helpful In-Container Tips**
```
┌─────────────────────────────────────────────────────────────────────────────┐
│  You are about to enter the container. Try these commands:                 │
│                                                                             │
│  ros2 launch ros_gz_sim_demos diff_drive.launch.py   # Launch Gazebo       │
│  ros2 run demo_nodes_cpp talker                      # Test publisher      │
│  ros2 topic list                                     # List topics          │
│  rviz2                                               # Visualization        │
│                                                                             │
│  Press Ctrl+D or type exit to return to menu                               │
└─────────────────────────────────────────────────────────────────────────────┘
```

**5. Cross-Reference to new-terminal.sh (The Synergy!)**

*Inside the container, users see this tip:*
```
┌─ 💡 TIP: Need More Terminals? ────────────────────────────────────┐
│  Open a NEW terminal on your host and run:                        │
│                                                                    │
│    ./scripts/new-terminal.sh          (interactive menu)          │
│    ./scripts/new-terminal.sh 1        (quick connect)             │
│                                                                    │
│  This connects to THIS container in a new terminal!               │
└────────────────────────────────────────────────────────────────────┘
```
*This is thoughtful UX design - teaching users without overwhelming them*

**6. Additional Menu Options**
- `r` - Refresh menu (checks if new images built)
- `s` - System status (running containers, GPU usage)
- `c` - Cleanup (stop all workshop containers)
- `q` - Quit

---

#### 9.2 new-terminal.sh: Simplicity Itself

**File:** `scripts/new-terminal.sh` (87 lines)

**The Innovation:** Abstracts `docker exec` complexity into user-friendly patterns.

**Usage Patterns:**

```bash
# Pattern 1: Interactive menu (no args needed)
./scripts/new-terminal.sh
# Shows list of running containers, user selects

# Pattern 2: Quick connect by number
./scripts/new-terminal.sh 1      # Opens terminal in workshop3-dds
./scripts/new-terminal.sh 2      # Opens terminal in workshop3-jazzy

# Pattern 3: Fuzzy name matching
./scripts/new-terminal.sh dds    # Matches "roscon-workshop3-dds"
./scripts/new-terminal.sh jazzy  # Matches "roscon-workshop3-jazzy"
./scripts/new-terminal.sh imu    # Matches "roscon-workshop4-imu"

# Pattern 4: Auto-connect if only one running
./scripts/new-terminal.sh
# Only one container running? Connect immediately!
```

**Smart Behavior:**
```bash
# Number mapping (lines 34-44):
1 → dds          (workshop3-dds)
2 → jazzy-zenoh  (workshop3-jazzy)
3 → humble-zenoh (workshop3-humble)
4 → imu          (workshop4-imu)
5 → robot-humble (robot)
6 → isaac-ros-base
7 → jazzy-base
```

**What It Replaces:**
```bash
# Old way (intimidating):
docker ps  # Find container ID or name
docker exec -it roscon-workshop3-dds bash
# Then source ROS manually
source /opt/ros/humble/setup.bash

# New way (friendly):
./scripts/new-terminal.sh 1
# ROS already sourced, consistent prompt, ready to go!
```

**Error Handling:**
```bash
# No containers running:
❌ No containers running. Start one with:
  ./scripts/launch-container.sh <number>

# Container name not found:
❌ No running container found matching: xyz
Available containers:
  - roscon-workshop3-dds
  - roscon-workshop3-jazzy
```

---

#### 9.3 The Bashrc Trick: Consistent Prompts Everywhere

**The Problem:**
- `launch-container.sh` launches with custom prompt
- `new-terminal.sh` should match that prompt
- Default `docker exec` gives ugly `root@abc123:/workspaces#`

**The Solution (Used in BOTH Scripts):**

```bash
# Create temporary bashrc (lines 364-374 in launch-container.sh)
cat > /tmp/.container_bashrc << 'ENDOFBASHRC'
# Source ROS automatically
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash 2>/dev/null

# Enable history persistence
shopt -s histappend
PROMPT_COMMAND='history -a'
history -r

# Beautiful, consistent prompt
PS1='\[\e[1;36m\][\h]\[\e[0m\] \[\e[1;34m\]\w\[\e[0m\] \$ '
ENDOFBASHRC

# Start bash with custom bashrc
exec bash --rcfile /tmp/.container_bashrc
```

**Result:**
```bash
# launch-container.sh launches with:
[workshop3-dds] /workspaces $

# new-terminal.sh connects with:
[workshop3-dds] /workspaces $

# Same prompt! User never confused about which container they're in
```

**Additional Benefits:**
- ✅ ROS 2 sourced automatically (no manual sourcing needed)
- ✅ Command history persists across terminals
- ✅ Colored prompt (cyan hostname, blue path)
- ✅ Works in Humble AND Jazzy (fallback logic)

---

#### 9.4 Why This Matters for Blog

**User Experience Transformation:**

| Aspect | Without Scripts | With Scripts |
|--------|----------------|--------------|
| **Launch container** | 15+ line docker command | `./launch-container.sh 1` |
| **New terminal** | `docker ps`, copy ID, `docker exec -it ...` | `./new-terminal.sh 1` |
| **Know which container** | Check hostname, confusing hash | Colored prompt shows name |
| **Source ROS** | Manual every terminal | Automatic via bashrc |
| **Check GPU available** | Launch and hope | Pre-flight checks before launch |
| **Learning curve** | Steep (Docker + ROS 2) | Gentle (guided experience) |

**Workshop Impact:**
- **Reduced instructor load**: Fewer "how do I...?" questions
- **Faster onboarding**: Participants productive in 5 minutes, not 30
- **Confidence building**: Success early → willing to explore more
- **Professional impression**: "Wow, this is well thought out!"

**Code Quality Indicators:**
- 500 lines for launch script (attention to detail)
- Color-coded everything (visual clarity)
- Error messages suggest solutions (helpful, not cryptic)
- Cross-references between scripts (ecosystem thinking)
- Consistent bashrc pattern (reusable across tools)

---

#### 9.5 Blog Code Snippets to Include

**Snippet 1: ASCII Banner Function** (showcases professionalism)
```bash
print_banner() {
    echo -e "${CYAN}"
    echo "╔═══════════════════════════════════════════════════════════════════════════════╗"
    echo "║   ██████╗  ██████╗ ███████╗ ██████╗ ██████╗ ███╗   ██╗    ██╗███╗   ██╗      ║"
    # ... (full banner)
    echo "╚═══════════════════════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}
```

**Snippet 2: Status Indicator Logic** (shows image availability)
```bash
# Check if image exists
if docker image inspect "$image" > /dev/null 2>&1; then
    local status="${GREEN}●${NC}"
    local size=$(docker image inspect "$image" --format='{{.Size}}' | \
                 awk '{printf "%.1fGB", $1/1024/1024/1024}')
else
    local status="${RED}○${NC}"
    local size="not built"
fi
```

**Snippet 3: new-terminal.sh Number Mapping** (fuzzy matching)
```bash
# Map number to container name
if [[ "$TARGET" =~ ^[0-9]+$ ]]; then
    case "$TARGET" in
        1) TARGET="dds" ;;
        2) TARGET="jazzy-zenoh" ;;
        # ...
    esac
fi

# Fuzzy match container name
CONTAINER_NAME=$(docker ps --filter "name=roscon-" --format "{{.Names}}" | \
                 grep -i "$TARGET" | head -1)
```

**Snippet 4: Consistent Bashrc** (reusable pattern)
```bash
cat > /tmp/.container_bashrc << 'ENDOFBASHRC'
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash 2>/dev/null
PS1='\[\e[1;36m\][\h]\[\e[0m\] \[\e[1;34m\]\w\[\e[0m\] \$ '
ENDOFBASHRC

exec bash --rcfile /tmp/.container_bashrc
```

---

#### 9.6 Screenshot to Include

**File:** `screenshots/launch-menu.png`

**Capture:** Full menu showing:
- ASCII banner at top
- 7 containers with status indicators (green ● / red ○)
- Image sizes displayed
- Menu options (r/s/c/q)
- Pre-flight check results

**Caption:**
> *launch-container.sh interactive menu: Professional ASCII art, color-coded status indicators (green = ready, red = not built), and pre-flight system checks. Transforms 15+ line Docker commands into a single keystroke.*

**Why This Screenshot Matters:**
- **Visual proof** of polish (not just code)
- **Instant credibility** - readers see professionalism
- **Shareworthy** - people will screenshot and share this
- **Demonstrates care** - attention to detail in UX

---

#### 9.7 Lessons for Other Workshop Organizers

**Pattern 1: Hide Complexity, Don't Eliminate It**
- Docker commands still run (not magic)
- But abstracted behind friendly interface
- Users can peek under the hood (scripts are readable)

**Pattern 2: Consistent Prompts Matter**
- Users across 7 containers, 2 ROS distros, 3 entry methods
- Same prompt everywhere = less cognitive load
- Small detail, huge UX impact

**Pattern 3: Cross-Reference Between Tools**
- launch-container tells users about new-terminal
- Scripts form an ecosystem, not isolated tools
- Guides users to discover features organically

**Pattern 4: Pre-flight Checks Save Time**
- Check Docker, GPU, X11 BEFORE launching
- Better to fail fast with helpful message
- Reduces "why isn't it working?" questions

**Pattern 5: Error Messages Suggest Solutions**
```bash
❌ Image not found!
Build it first with:
  cd ~/docker/ros2 && make
```
*Don't just say what's wrong - say how to fix it*

---

### 10. Documentation Strategy

### 10. Documentation Strategy

**Created Guides:**

1. **TURTLEBOT3_MIGRATION.md** (337 lines)
   - Command comparison table
   - 19 demo launch files
   - Topic name changes
   - Multiple robots example
   - FAQs (15 questions)
   - Docker usage patterns

2. **TROUBLESHOOTING.md** (572 lines)
   - 7 major sections
   - Docker & build issues
   - ROS 2 runtime errors
   - Gazebo simulation issues
   - Cache & offline issues
   - Zenoh networking
   - Container access
   - GPU/display

3. **README.md** - "Your First Demo"
   - 3-step beginner guide
   - Expected output description
   - Link to migration guide

**Structure for Reusability:**
- Problem → Solution format
- Copy/paste ready commands
- Screenshot descriptions
- "Why this works" explanations

### 11. Quality Assurance: Multi-Role Certification

**Process:** 8-role review before Phase 8 (offline prep)

**Roles:**
1. 🏗️ Software Architect
2. 🔧 DevOps Engineer
3. 🤖 ROS2 Developer
4. 🧪 QA/Test Engineer
5. 🔐 Security Analyst
6. 📚 Technical Writer
7. 📋 Project Manager
8. 👤 End User

**Round 1 Results:**
- 2/8 fully certified
- 4 CRITICAL blocking issues found
- Verdict: ⚠️ CONDITIONALLY CERTIFIED

**Blocking Issues:**
- T7.5-1: No runtime testing
- T7.5-2: No offline verification
- U7.5-3: No TurtleBot3 migration guide
- U7.5-6: Offline-save doesn't include cache

**Round 2 Results (After Fixes):**
- 8/8 fully certified ✅
- All 4 CRITICAL issues resolved
- Verdict: ✅ FULLY CERTIFIED - READY FOR PHASE 8

**Improvement:**
- QA/Tester: ❌ → ✅ (runtime + offline tests)
- End User: ❌ → ✅ (migration guide + quick start)

**Process Value:**
- Caught offline failures before workshop
- Prevented teaching deprecated technology
- Ensured documentation completeness

### 12. Lessons Learned

#### What Worked Well

1. **Three-Tier Strategy:**
   - Clear separation of concerns
   - Fast rebuilds (cached layers)
   - Easy to maintain

2. **Pause Before Phase 8:**
   - Caught TurtleBot3 deprecation in time
   - Avoided 80GB of wrong images
   - Saved weeks of rework

3. **Shared Cache Pattern:**
   - Discovered organically (user question)
   - Applicable beyond Gazebo (pip, colcon)
   - 75% disk savings

4. **Multi-Role Certification:**
   - Caught missing offline tests
   - Ensured end-user perspective
   - Quality gates before major milestones

5. **Documentation First:**
   - README, migration guide, troubleshooting
   - Participants can self-serve
   - Reduces instructor load

#### What We'd Do Differently

1. **Check for EOL Earlier:**
   - Research software lifecycles in Phase 1
   - Add to architecture review checklist
   - Saved time if caught at Phase 2

2. **Test Offline Sooner:**
   - Should have verified offline in Phase 6
   - Don't wait until Phase 7.5
   - Add to smoke tests

3. **Document Naming Conventions:**
   - `ign` vs `gz` caught us off guard
   - Should have researched ROS 2 distro differences
   - Created compatibility matrix

4. **Parallel Agent Usage:**
   - Used agents more in Phase 7.5
   - Could have used agents from Phase 1
   - Preserves context, faster execution

#### Known Limitations

1. **workshop3-humble-zenoh:**
   - Status: ❌ Broken
   - Issue: Needs Rust/Cargo for rmw_zenoh source build
   - Workaround: Use workshop3-humble-dds instead
   - Fix: Install Rust in isaac-ros-base (adds 2GB)

2. **Fuel Cache Not Required:**
   - Initial plan: Pre-populate Fuel cache
   - Reality: ros-gz-sim-demos bundles models
   - Impact: Shared cache is optional (good to have)
   - Benefit: Simpler offline story

3. **Nav2 Launch Files Missing (Humble):**
   - Jazzy: 2 launch files (tb3, tb4 loopback)
   - Humble: 0 launch files
   - Reason: Different package structure
   - Workaround: Use individual nodes

4. **Lifecycle Manager Needs Config:**
   - Error: `parameter 'node_names' is not initialized`
   - Expected: Needs YAML config file
   - Not blocking: Individual nodes work fine

#### Unexpected Discoveries

1. **Models Bundled in Package:**
   - Eliminated Fuel dependency for demos
   - Simpler offline operation
   - Better than planned architecture

2. **Ignition vs Gazebo Naming:**
   - Humble pre-dates rebranding
   - Different cache paths
   - Different CLI commands

3. **Nav2 Already Complete:**
   - Thought we only had partial Nav2
   - Actually had full stack (30-34 packages)
   - Just needed to verify

4. **ROS_LOCALHOST_ONLY Discovery (Phase 7.6):**
   - Same base image, different behavior
   - Environment variables override Dockerfile defaults
   - Critical for topic discovery in containers
   - Debugging required systematic hypothesis testing

5. **HID Devices Need Special Permissions (Phase 7.6):**
   - USB passthrough ≠ HID device access
   - `device_cgroup_rules` pattern enables IMU access
   - Character device class 13 = HID
   - Not documented in most Docker tutorials

6. **Volume Mount Override Pattern (Phase 7.6):**
   - Can fix upstream launch file bugs without rebuilding
   - More maintainable than patching packages
   - Elegant solution to namespace collision

7. **RTX 5090 Forward Compatibility (Phase 7.7):**
   - Driver 570.86.16 supports Blackwell (Compute 12.0)
   - CUDA 12.2 containers work via driver runtime
   - Test hardware compatibility, don't assume from docs
   - Bleeding-edge GPUs supported before official docs

8. **Network Bandwidth Debugging (Phase 7.7):**
   - Created `/proc/net/dev` monitoring script
   - 12.2GB download at 23 MB/s isn't "stuck"
   - Always check bandwidth before assuming failure
   - Reusable pattern for future downloads

### 13. Try It Yourself

**Prerequisites:**
- Ubuntu 22.04 or 24.04
- Docker + Docker Compose
- NVIDIA GPU (optional, but recommended)
- 200GB disk space
- 16GB+ RAM

**Quick Start:**

1. **Clone Repository:**
   ```bash
   # Blog repo (sanitized, public)
   git clone https://github.com/yourusername/roscon-india-2025-workshop-blog.git
   cd roscon-india-2025-workshop-blog
   ```

2. **Build Images:**
   ```bash
   make all
   # First build: ~60-90 minutes
   # Rebuilds: ~5-10 minutes
   ```

3. **Run Your First Demo:**
   ```bash
   docker compose run --rm workshop3-jazzy bash
   source /opt/ros/jazzy/setup.bash
   ros2 launch ros_gz_sim_demos diff_drive.launch.py
   ```

4. **Control the Robot:**
   ```bash
   # New terminal
   ./scripts/new-terminal.sh
   ros2 topic pub /model/vehicle_blue/cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.5}, angular: {z: 0.3}}" -r 10
   ```

**What You'll See:**
- Gazebo Sim window with 2 robots (blue + green)
- RViz2 visualization
- Terminal showing topic bridging

**Troubleshooting:**
See `TROUBLESHOOTING.md` in the repo.

**Offline Usage:**
```bash
# Save images (before workshop)
make offline-save

# Load images (at workshop venue, no internet)
make offline-load

# Verify offline
docker run --network none jazzy-base:latest bash -c \
  "ros2 launch ros_gz_sim_demos diff_drive.launch.py"
```

### 14. Adapting for Your Workshop

**Generic Template Checklist:**

1. **Choose ROS 2 Distro:**
   - Humble (LTS, until 2027)
   - Jazzy (Latest LTS, until 2029)
   - Consider participant hardware

2. **Select Base Image:**
   - GPU needed? → NVIDIA Isaac ROS
   - CPU only? → OSRF official images
   - Custom hardware? → Build from scratch

3. **Middleware Decision:**
   - WiFi workshop? → Zenoh or bridge
   - Wired network? → CycloneDDS/FastDDS
   - Multi-robot? → Unique ROS_DOMAIN_IDs

4. **Simulation Platform:**
   - Gazebo Sim (recommended, modern)
   - Gazebo Classic (EOL Jan 2025, avoid!)
   - Custom (MuJoCo, Isaac Sim, etc.)

5. **Offline Strategy:**
   - Pre-install packages in Dockerfile
   - Use shared caches (pip, colcon, Fuel)
   - Save images to USB drives
   - Test with `--network none`

6. **Documentation:**
   - Create README with quick start
   - Write troubleshooting guide (use ours as template)
   - Migration guides if changing from other tools
   - Record asciinema demos

7. **Quality Gates:**
   - Smoke tests (make test)
   - Offline tests (disconnect network)
   - Multi-role review (use our template)
   - Participant beta test (before event)

**Timing:**
- Start 4-6 weeks before workshop
- Phase 1-6: 2-3 weeks (architecture + build)
- Phase 7: 1 week (testing + fixes)
- Phase 8: 3-5 days (offline prep)
- Buffer: 1 week (unexpected issues)

**Budget:**
- Disk space: 200GB+ for images
- USB drives: 128GB per participant (optional)
- Build time: ~60-90 min initial, ~5-10 min rebuilds
- Test time: 1-2 days thorough testing

### 15. Future Improvements

**Considered But Deferred:**

1. **VNC/noVNC for Remote Access:**
   - Why: Browser-based access to containers
   - Deferred: Workshop has local laptops
   - Future: Online workshops need this

2. **Kubernetes Orchestration:**
   - Why: Multi-node workshop coordination
   - Deferred: Docker Compose sufficient for 100 participants
   - Future: >500 participants need K8s

3. **Custom rmw_zenoh Build:**
   - Why: workshop3-humble-zenoh currently broken
   - Deferred: workshop3-humble-dds works (bridge approach)
   - Future: Rust toolchain in isaac-ros-base

4. **Automated Screenshot Testing:**
   - Why: Verify GUIs render correctly
   - Deferred: Manual verification worked
   - Future: CI/CD pipeline needs this

5. **Model Optimization:**
   - Why: Smaller robot models = faster sim
   - Deferred: ros-gz-sim-demos models work fine
   - Future: Custom curriculum needs custom models

**Roadmap:**

- **Short-term** (next workshop):
  - Add VNC support
  - Fix workshop3-humble-zenoh (Rust)
  - More Nav2 launch file examples

- **Medium-term** (6 months):
  - Create generic workshop template repo
  - Add CI/CD for image builds
  - Automated testing framework

- **Long-term** (1 year):
  - Kubernetes deployment option
  - Web-based management UI
  - Multi-site workshop coordination

### 16. Acknowledgments

**Tools Used:**
- Docker + BuildKit
- ROS 2 (Humble, Jazzy)
- Gazebo Sim (Ignition Fortress, Gazebo Sim)
- Zenoh (Eclipse Foundation)
- NVIDIA Isaac ROS
- Claude Code (Anthropic)

**Community:**
- Open Robotics (ros-gz-sim-demos)
- NVIDIA (Isaac ROS platform)
- Eclipse Zenoh team
- ROSCon India 2025 organizers

### 17. Conclusion

**Summary:**

We built a production-grade offline workshop infrastructure with:
- 7 Docker images (150GB → 80-95GB compressed)
- Modern Gazebo Sim (avoided EOL trap)
- GPU acceleration + Zenoh bridging
- Complete Nav2 stack
- Comprehensive documentation
- Multi-role quality certification

**Key Success Factors:**

1. **Pause to Validate:** Catching TurtleBot3 EOL at Phase 7 saved weeks
2. **User-Centric:** Scripts hide Docker complexity from participants
3. **Offline-First:** Every decision considered "what if no internet?"
4. **Documentation:** 3 guides (migration, troubleshooting, quick start)
5. **Quality Gates:** Multi-role certification prevented workshop failures

**Timeline:**
- Dec 14: Phase 1-6 complete (architecture + build)
- Dec 15 AM: Phase 7.5 modernization (TurtleBot3 → ros-gz-sim-demos)
- Dec 15 AM: Multi-role certification (round 1 → round 2)
- Dec 15 PM: Phase 7.6 RealSense D435i + VSLAM deployment (3 hidden issues debugged)
- Dec 15 PM: Phase 7.7 RTX 5090 hardware verification (Blackwell GPU compatibility confirmed)
- Dec 16-17: Phase 8 offline prep (pending)
- Dec 18-20: ROSCon India 2025 workshop

**Reusability:**

This infrastructure is **not** ROSCon India specific. The patterns apply to:
- University robotics courses
- Corporate training workshops
- Conference hands-on sessions
- Research lab onboarding
- Open-source project demos

Use this blog post as a blueprint. Adapt the Dockerfiles, configs, and scripts to your needs. The architecture principles remain valid.

**Final Thought:**

Building workshop infrastructure is like building a robot: you discover problems during testing, not during deployment. We caught TurtleBot3's EOL issue at Phase 7 because we paused to test. The multi-role certification caught offline failures before the workshop.

**Pause. Test. Certify. Then deploy.**

---

## Source Files to Reference

| Content | Source File (Private Repo) | Blog Equivalent |
|---------|----------------------------|-----------------|
| Architecture decisions | `PLAN.md` | Inline in blog post |
| Test results | `PROGRESS.md` | Inline in blog post |
| Migration guide | `TURTLEBOT3_MIGRATION.md` | Copy to blog repo (as example) |
| Troubleshooting | `TROUBLESHOOTING.md` | Copy to blog repo |
| Certification process | `PHASE7.5_CERTIFICATION.md` + `ROUND2.md` | Summarize in blog |
| Base Dockerfile | `images/isaac-ros-base/Dockerfile` | Key snippets only |
| Docker Compose | `docker-compose.yml` | Template version |
| Makefile | `Makefile` | Key targets as markdown |
| Launch script | `scripts/launch-container.sh` | Core snippet only |
| CycloneDDS config | `configs/cyclonedds.xml` | Full file (safe) |
| Zenoh config | `configs/zenoh/config.json5` | Full file (safe) |

---

## Visual Content Strategy (UPDATED - Hybrid Approach)

**Strategy:** Use **code blocks for terminal output** (preferred for technical blogs) + **optional GUI screenshots** for visual impact.

### Terminal Outputs (As Code Blocks) - 11 items ✅

These will be captured programmatically and included as syntax-highlighted code blocks:

1. **rtx5090-nvidia-smi** - `nvidia-smi` output showing RTX 5090 + driver 570.86.16
2. **pytorch-cuda-test** - PyTorch CUDA 12.8 test (Compute 12.0 detected)
3. **bandwidth-monitor** - `/tmp/check_bandwidth.sh` output (23 MB/s)
4. **makefile-help** - `make help` showing all targets
5. **realsense-imu-output** - `rs-enumerate-devices` showing D435i with IMU
6. **vslam-test-results** - VSLAM test script output (3/3 containers PASS)
7. **container-prompt** - Custom prompt `[workshop3-dds] /workspaces $`
8. **ros-localhost-only-debug** - Environment variable comparison
9. **nav2-nodes** - `ros2 pkg list | grep nav2` output
10. **bridge-test-terminal** - zenoh-bridge-ros2dds success output
11. **launch-menu** - ASCII banner and menu (can be rendered as text block)

**Benefits of Code Blocks:**
- ✅ Copy-pasteable for readers
- ✅ Searchable by search engines
- ✅ No compression artifacts
- ✅ Accessible for screen readers
- ✅ Smaller file size

### GUI Screenshots (Optional - Can Add Later) - 5 items 📸

If needed for visual impact, capture these manually:

12. **gazebo-sim-diff-drive.png** - Two robots (blue + green) in Gazebo Sim
13. **rviz2-demo.png** - RViz2 showing robot model + transforms
14. **new-terminal-usage.png** - Interactive menu screenshot
15. **vslam-namespace-fix.png** - Before/after diagram (can be ASCII diagram instead)

**Alternative:** Use stock images from official ROS/Gazebo documentation with proper attribution.

### Current Status:
- ✅ Thumbnail: Generated and saved (Gemini AI illustration)
- 🔄 Terminal outputs: Will be generated by blog creation agent
- ⏸️ GUI screenshots: Deferred (blog is strong without them, can add in future update)

---

## Code Snippets to Extract (Sanitized)

### 1. Dockerfile Snippet: isaac-ros-base (Key Sections)

```dockerfile
# File: code/Dockerfile-snippet-isaac-ros.dockerfile
# ROSCon India 2025 Workshop - Isaac ROS Base Image (Key Sections)

# ═══════════════════════════════════════════════════════════════════════════════
# TIER 1: NVIDIA Isaac ROS Base
# ═══════════════════════════════════════════════════════════════════════════════
FROM nvcr.io/nvidia/isaac/ros:x86_64-ros2_humble_<hash> AS base

# ═══════════════════════════════════════════════════════════════════════════════
# TIER 2: Custom Base with Modern Gazebo
# ═══════════════════════════════════════════════════════════════════════════════

# Install modern Gazebo + ros-gz-sim-demos (NOT TurtleBot3!)
RUN apt-get update && apt-get install -y \
    # Complete Nav2 stack
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    # Modern Gazebo simulation (not Classic - EOL Jan 2025)
    ros-humble-ros-gz-sim-demos \
    && rm -rf /var/lib/apt/lists/*

# Note: Models bundled in package, no Fuel download needed!
# /opt/ros/humble/share/ros_gz_sim_demos/models/vehicle/

# ═══════════════════════════════════════════════════════════════════════════════
# Entrypoint for ROS 2 environment sourcing
# ═══════════════════════════════════════════════════════════════════════════════
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
```

### 2. Docker Compose Template

```yaml
# File: code/docker-compose-template.yml
# ROSCon India 2025 Workshop - Docker Compose Template

services:
  workshop3-jazzy:
    build:
      context: ./images/jazzy-base
      dockerfile: Dockerfile
    image: jazzy-base:latest
    container_name: workshop3-jazzy
    hostname: workshop3-jazzy

    # GPU passthrough (if available)
    runtime: nvidia
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=20

    # Network mode for ROS 2 discovery
    network_mode: host
    privileged: true

    volumes:
      # X11 for GUI apps
      - /tmp/.X11-unix:/tmp/.X11-unix:rw

      # Shared caches (download once, use everywhere)
      - ./cache/pip:/root/.cache/pip:rw
      - ./cache/colcon:/root/.colcon:rw
      - ./cache/gz:/root/.gz:rw                    # Gazebo Fuel (Jazzy)

      # Persistent workspace
      - ./workspaces/workshop3-jazzy:/workspaces:rw

      # Configurations (read-only)
      - ./configs/zenoh/config.json5:/config/zenoh.json5:ro

    # Interactive shell by default
    stdin_open: true
    tty: true
```

### 3. Makefile Key Targets

```makefile
# File: code/Makefile-key-targets.md
# ROSCon India 2025 Workshop - Key Makefile Targets

# Build everything
all: base workshop3 workshop4 robot

# Build base images (TIER 2)
base: base-isaac base-jazzy

base-isaac:
	docker build -t isaac-ros-base:humble ./images/isaac-ros-base

base-jazzy:
	docker build -t jazzy-base:latest ./images/jazzy-base

# Build workshop images (TIER 3)
workshop3:
	docker compose build workshop3-dds workshop3-jazzy workshop3-humble

# Test images
test: test-dds test-jazzy test-imu

test-dds:
	docker run --rm workshop3-humble-dds bash -c \
	  "source /opt/ros/humble/setup.bash && ros2 topic list"

# Offline preparation (saves to ./offline/)
offline-save:
	mkdir -p offline
	docker save isaac-ros-base:humble | zstd -T0 -19 > offline/isaac-ros-base.tar.zst
	docker save jazzy-base:latest | zstd -T0 -19 > offline/jazzy-base.tar.zst
	# ... (saves all 7 images)
	tar -czf offline/gazebo-fuel-cache.tar.gz cache/  # Include caches

offline-load:
	zstd -d -c offline/isaac-ros-base.tar.zst | docker load
	zstd -d -c offline/jazzy-base.tar.zst | docker load
	# ... (loads all 7 images)
	tar -xzf offline/gazebo-fuel-cache.tar.gz  # Extract caches
```

### 4. CycloneDDS Configuration (Full File - Safe)

```xml
<!-- File: code/cyclonedds.xml -->
<!-- ROSCon India 2025 Workshop - CycloneDDS Configuration -->
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain id="any">
    <General>
      <!-- Auto-detect network interface -->
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>

      <!-- Disable multicast (WiFi workshops) -->
      <AllowMulticast>false</AllowMulticast>

      <!-- Use unicast discovery instead -->
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>

    <Discovery>
      <EnableTopicDiscoveryEndpoints>true</EnableTopicDiscoveryEndpoints>

      <!-- Faster discovery for workshop demo -->
      <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>

    <Internal>
      <!-- Optimize for local network -->
      <MaxSampleSize>65500B</MaxSampleSize>
    </Internal>
  </Domain>
</CycloneDDS>
```

### 5. Offline Save Script

```bash
#!/bin/bash
# File: code/offline-save-snippet.sh
# ROSCon India 2025 Workshop - Offline Image Saving

set -e

OFFLINE_DIR="offline"
mkdir -p "$OFFLINE_DIR"

echo "Saving Docker images for offline workshop..."

# Save base images
echo "Saving isaac-ros-base..."
docker save isaac-ros-base:humble | zstd -T0 -19 > "$OFFLINE_DIR/isaac-ros-base.tar.zst"

echo "Saving jazzy-base..."
docker save jazzy-base:latest | zstd -T0 -19 > "$OFFLINE_DIR/jazzy-base.tar.zst"

# Save workshop images
for img in workshop3-dds workshop3-jazzy workshop4-imu robot-humble; do
    echo "Saving $img..."
    docker save "$img:latest" | zstd -T0 -19 > "$OFFLINE_DIR/$img.tar.zst"
done

# Archive shared caches
if [ -d "cache" ]; then
    echo "Archiving Gazebo Fuel cache..."
    tar -czf "$OFFLINE_DIR/gazebo-fuel-cache.tar.gz" cache/
fi

# Summary
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✅ Offline images saved to $OFFLINE_DIR/"
du -sh "$OFFLINE_DIR"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
```

---

### 6. Launch Container Snippets (NEW)

```bash
#!/bin/bash
# File: code/launch-container-snippets.sh
# ROSCon India 2025 Workshop - Key Functions from Interactive Launcher
# Full script: 500 lines. These are the most interesting/reusable patterns.

# ═══════════════════════════════════════════════════════════════════════════════
# Professional ASCII Banner
# ═══════════════════════════════════════════════════════════════════════════════
print_banner() {
    echo -e "${CYAN}"
    echo "╔═══════════════════════════════════════════════════════════════════════════════╗"
    echo "║                                                                               ║"
    echo "║   ██████╗  ██████╗ ███████╗ ██████╗ ██████╗ ███╗   ██╗    ██╗███╗   ██╗      ║"
    echo "║   ██╔══██╗██╔═══██╗██╔════╝██╔════╝██╔═══██╗████╗  ██║    ██║████╗  ██║      ║"
    echo "║   ██████╔╝██║   ██║███████╗██║     ██║   ██║██╔██╗ ██║    ██║██╔██╗ ██║      ║"
    echo "║   ██╔══██╗██║   ██║╚════██║██║     ██║   ██║██║╚██╗██║    ██║██║╚██╗██║      ║"
    echo "║   ██║  ██║╚██████╔╝███████║╚██████╗╚██████╔╝██║ ╚████║    ██║██║ ╚████║      ║"
    echo "║   ╚═╝  ╚═╝ ╚═════╝ ╚══════╝ ╚═════╝ ╚═════╝ ╚═╝  ╚═══╝    ╚═╝╚═╝  ╚═══╝      ║"
    echo "║                                                                               ║"
    echo "║                    India 2025 Workshop - Container Launcher                   ║"
    echo "║                         December 18-20, COEP Pune                             ║"
    echo "║                                                                               ║"
    echo "╚═══════════════════════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

# ═══════════════════════════════════════════════════════════════════════════════
# Image Status Indicator (Green = Built, Red = Not Built)
# ═══════════════════════════════════════════════════════════════════════════════
check_image_status() {
    local image=$1

    if docker image inspect "$image" > /dev/null 2>&1; then
        # Image exists - green indicator
        local status="${GREEN}●${NC}"
        local size=$(docker image inspect "$image" --format='{{.Size}}' 2>/dev/null | \
                     awk '{printf "%.1fGB", $1/1024/1024/1024}')
    else
        # Image missing - red indicator
        local status="${RED}○${NC}"
        local size="not built"
    fi

    echo "$status $size"
}

# ═══════════════════════════════════════════════════════════════════════════════
# Pre-flight System Checks (Before Launching Container)
# ═══════════════════════════════════════════════════════════════════════════════
preflight_checks() {
    print_header "System Checks"

    # Check Docker
    if command -v docker &> /dev/null; then
        local docker_version=$(docker --version | cut -d' ' -f3 | tr -d ',')
        print_success "Docker $docker_version"
    else
        print_error "Docker not installed"
        return 1
    fi

    # Check NVIDIA runtime
    if docker info 2>/dev/null | grep -q "nvidia"; then
        print_success "NVIDIA Container Toolkit"
    else
        print_warning "NVIDIA runtime not detected"
    fi

    # Check X11 display
    if [ -n "$DISPLAY" ]; then
        print_success "Display: $DISPLAY"
    else
        print_error "DISPLAY not set"
        return 1
    fi

    # Check GPU
    if command -v nvidia-smi &> /dev/null; then
        local gpu_name=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)
        print_success "GPU: $gpu_name"
    fi

    # Allow X11 connections from Docker
    xhost +local:docker > /dev/null 2>&1
}

# ═══════════════════════════════════════════════════════════════════════════════
# Consistent Bashrc Pattern (Works Across All Containers)
# ═══════════════════════════════════════════════════════════════════════════════
setup_container_bashrc() {
    # Creates /tmp/.container_bashrc inside container
    # Used by both launch-container.sh and new-terminal.sh for consistency

    cat > /tmp/.container_bashrc << 'ENDOFBASHRC'
# Auto-source ROS (Humble or Jazzy)
source /opt/ros/humble/setup.bash 2>/dev/null || \
source /opt/ros/jazzy/setup.bash 2>/dev/null

# Enable persistent history
shopt -s histappend
PROMPT_COMMAND='history -a'
history -r

# Colorful, informative prompt: [container-name] /path $
PS1='\[\e[1;36m\][\h]\[\e[0m\] \[\e[1;34m\]\w\[\e[0m\] \$ '
ENDOFBASHRC

    # Launch bash with custom config
    exec bash --rcfile /tmp/.container_bashrc
}

# ═══════════════════════════════════════════════════════════════════════════════
# User-Friendly Error Messages (Always Suggest Solution)
# ═══════════════════════════════════════════════════════════════════════════════
handle_missing_image() {
    local image=$1

    print_error "Image '$image' not found!"
    echo ""
    echo "Build it first with:"
    echo -e "  ${GREEN}cd ${PROJECT_DIR} && make${NC}"
    echo ""
    read -p "Press Enter to return to menu..."
    return 1
}
```

---

### 7. new-terminal.sh (Full Script - NEW)

```bash
#!/bin/bash
# File: code/new-terminal.sh
# ROSCon India 2025 Workshop - Open Additional Terminals in Running Containers
# This is the FULL script (87 lines) - small enough to include entirely

set -e

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}  Exec into Running Container${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# Parse command-line argument
TARGET="$1"

# List running containers
echo -e "${YELLOW}Running containers:${NC}"
docker ps --format "table {{.Names}}\t{{.Image}}\t{{.Status}}" | grep -E "(NAMES|roscon-)" || echo "No containers running"

echo ""

# Map number to container name (for convenience)
if [[ "$TARGET" =~ ^[0-9]+$ ]]; then
    case "$TARGET" in
        1) TARGET="dds" ;;
        2) TARGET="jazzy-zenoh" ;;
        3) TARGET="humble-zenoh" ;;
        4) TARGET="imu" ;;
        5) TARGET="robot-humble" ;;
        6) TARGET="isaac-ros-base" ;;
        7) TARGET="jazzy-base" ;;
    esac
fi

# If target specified, try fuzzy matching
if [ -n "$TARGET" ]; then
    CONTAINER_NAME=$(docker ps --filter "name=roscon-" --format "{{.Names}}" | grep -i "$TARGET" | head -1)

    if [ -z "$CONTAINER_NAME" ]; then
        echo -e "${RED}No running container found matching: $TARGET${NC}"
        echo -e "${YELLOW}Available containers:${NC}"
        docker ps --filter "name=roscon-" --format "  - {{.Names}}"
        exit 1
    fi

    echo -e "${GREEN}Connecting to: ${CONTAINER_NAME}${NC}"
    echo ""
    docker exec -it "$CONTAINER_NAME" bash --rcfile /tmp/.container_bashrc
    exit 0
fi

# No target specified - interactive mode
CONTAINER_COUNT=$(docker ps --filter "name=roscon-" --format "{{.Names}}" | wc -l)

if [ "$CONTAINER_COUNT" -eq 0 ]; then
    echo -e "${RED}No containers running. Start one with:${NC}"
    echo "  ./scripts/launch-container.sh <number>"
    exit 1
elif [ "$CONTAINER_COUNT" -eq 1 ]; then
    # Only one container - auto-connect
    CONTAINER_NAME=$(docker ps --filter "name=roscon-" --format "{{.Names}}")
    echo -e "${GREEN}Connecting to: ${CONTAINER_NAME}${NC}"
    echo ""
    docker exec -it "$CONTAINER_NAME" bash --rcfile /tmp/.container_bashrc
else
    # Multiple containers - ask which one
    echo -e "${YELLOW}Multiple containers running. Which one?${NC}"
    select CONTAINER_NAME in $(docker ps --filter "name=roscon-" --format "{{.Names}}"); do
        if [ -n "$CONTAINER_NAME" ]; then
            echo -e "${GREEN}Connecting to: ${CONTAINER_NAME}${NC}"
            echo ""
            docker exec -it "$CONTAINER_NAME" bash --rcfile /tmp/.container_bashrc
            break
        fi
    done
fi
```

**Why Include Full Script:**
- Only 87 lines (small enough for blog)
- Demonstrates all 4 usage patterns (interactive, number, fuzzy, auto)
- Shows clean bash practices (error handling, user feedback)
- Readers can copy/paste and adapt for their workshops

---

## Security Review Checklist (MANDATORY)

Before publishing, verify NO secrets are exposed:

### ✅ Safe to Include
- [x] Package versions and apt commands
- [x] Build commands (docker build, make)
- [x] Architecture diagrams (ASCII, images)
- [x] Generic configuration patterns
- [x] CycloneDDS XML (no secrets)
- [x] Zenoh config JSON (no tokens)
- [x] Screenshot of terminal output (no credentials visible)

### ❌ Never Include
- [ ] Git PAT tokens (`~/.git-credentials` content)
- [ ] Claude API keys (`configs/claude/settings.json` with real keys)
- [ ] SSH private keys or paths
- [ ] Docker Hub registry credentials
- [ ] Real IP addresses (use 192.168.x.x examples)
- [ ] Real hostnames (use generic names)
- [ ] WiFi passwords
- [ ] Full paths with usernames (`/home/rajesh/` → `${HOME}/`)

### Redaction Patterns

```bash
# ✅ SAFE: Show the pattern
-v ${HOME}/.git-credentials:/root/.git-credentials:ro

# ❌ UNSAFE: Never show actual content
# github.com=ghp_XXXXXXXXXXXXXXXXXX  # NEVER INCLUDE THIS!

# ✅ SAFE: Generic example
export GITHUB_TOKEN="<your-github-token>"

# ✅ SAFE: Placeholder
CLAUDE_API_KEY="sk-ant-api03-..."  # Replace with your key
```

---

## Post-Creation Tasks

1. [ ] Create post directory: `mkdir -p blog/posts/2025-12-15-roscon-india-workshop-docker-infrastructure/{screenshots,code}`
2. [ ] Capture screenshots (7 images, <1MB each)
3. [ ] Extract code snippets (sanitize paths, remove secrets)
4. [ ] Write main post (`index.qmd`)
5. [ ] **SECURITY REVIEW**: Complete all checklist items
6. [ ] Create thumbnail image (Gazebo Sim screenshot, cropped)
7. [ ] Update `blog/_quarto.yml` sidebar
8. [ ] Preview: `quarto preview blog/`
9. [ ] Verify all links work
10. [ ] Verify code snippets render correctly
11. [ ] Copy TURTLEBOT3_MIGRATION.md to blog repo
12. [ ] Copy TROUBLESHOOTING.md to blog repo
13. [ ] Commit to blog repo
14. [ ] Push to GitHub
15. [ ] Render and deploy: `quarto render && quarto publish`

---

## Estimated Stats (UPDATED after Phase 7.6-7.7)

- **Target Length**: 5500-6500 words (expanded from original 4000-5000)
- **Reading Time**: 25-30 minutes
- **Code Blocks**: 20-25 (increased due to debugging sections)
- **Diagrams**: 4-5
- **Screenshots**: 9 (added VSLAM test results + RTX 5090 verification)
- **Sections**: 17 (vs original 11)
- **New Content**: ~50% (Phase 7.5 modernization + Phase 7.6 VSLAM debugging + Phase 7.7 RTX 5090 verification)

**Content Breakdown:**
- **Phase 7.5**: TurtleBot3 → ros-gz-sim-demos migration (~800 words)
- **Phase 7.6**: RealSense D435i + VSLAM debugging (~1200 words)
- **Phase 7.7**: RTX 5090 hardware verification (~900 words)
- **User Experience Scripts**: launch-container.sh + new-terminal.sh (~600 words)
- **Architecture & Build**: ~1500 words
- **Documentation & QA**: ~800 words
- **Lessons Learned**: ~700 words (expanded with 5 new discoveries)

**Key Additions in Phase 7.6-7.7:**
- Three debugging stories (IMU permissions, namespace collision, ROS_LOCALHOST_ONLY)
- RTX 5090 Blackwell architecture testing
- Network bandwidth monitoring script
- Systematic troubleshooting methodology
- Real-world failure modes and solutions

---

## Blog Repository Structure

```
roscon-india-2025-workshop-blog/  (PUBLIC repo)
├── README.md
├── code/
│   ├── Dockerfile-snippet-isaac-ros.dockerfile
│   ├── Dockerfile-snippet-jazzy.dockerfile
│   ├── docker-compose-template.yml
│   ├── cyclonedds.xml
│   ├── zenoh-config.json5
│   ├── Makefile-key-targets.md
│   └── offline-save-snippet.sh
├── docs/
│   ├── TURTLEBOT3_MIGRATION.md  (copied from private repo)
│   └── TROUBLESHOOTING.md       (copied from private repo)
├── screenshots/
│   ├── launch-menu.png
│   ├── gazebo-sim-diff-drive.png
│   ├── rviz2-demo.png
│   ├── bridge-test-terminal.png
│   ├── nav2-nodes.png
│   ├── offline-test.png
│   └── makefile-help.png
└── blog/
    └── index.qmd  (main blog post)
```

**Separation:**
- Private repo: Full Dockerfiles, workspaces, credentials
- Blog repo: Sanitized snippets, documentation, screenshots
- Blog post: Links to blog repo for code examples

---

**Last Updated:** 2025-12-15 (Phase 7.7 Complete - RTX 5090 Verified)
**Status:** Ready for blog post creation with Phase 7.6-7.7 content
**Next Step:** Capture screenshots (16 total), extract code snippets, write index.qmd

**Recent Additions:**
- Section 6.7: RealSense D435i + VSLAM debugging (3 issues: IMU permissions, namespace collision, ROS_LOCALHOST_ONLY)
- Section 6.8: RTX 5090 hardware verification (Blackwell Compute 12.0 compatibility, network bandwidth debugging)
- Updated timeline with Phase 7.6-7.7 entries
- Expanded lessons learned with 5 new discoveries
- Updated estimated stats: 5500-6500 words, 25-30 min read
- Added 7 new screenshots for Phase 7.6-7.7
