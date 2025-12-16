# Workshop 3 Part 3: Session Notes
## Exercises 6-8: Advanced Networking

> **Session Date**: December 16, 2025
> **Container Used**: Option 2 - `workshop3-jazzy` (Jazzy + rmw_zenoh_cpp)

---

## Exercise 6: Wireless Performance Tuning

### Setup Used
- RealSense D435i camera (720p @ 30 FPS)
- Zenoh router (`ros2 run rmw_zenoh_cpp rmw_zenohd`)
- Traffic Control (`tc`) to simulate WiFi congestion

### Baseline Measurements (No Network Limits)

```bash
# Terminal 1: Start router
ros2 run rmw_zenoh_cpp rmw_zenohd

# Terminal 2: Start camera
ros2 launch realsense2_camera rs_launch.py

# Terminal 3: Measure
ros2 topic bw /camera/camera/color/image_raw
ros2 topic hz /camera/camera/color/image_raw
```

**Results (no limits):**
```
Bandwidth: 84-86 MB/s
FPS: 30 FPS (perfect)
Message size: 2.76 MB per frame
Jitter (std dev): 0.0004s
```

### Kernel Buffer Settings (Default)
```bash
# Check current settings
sysctl net.core.rmem_max net.core.wmem_max net.core.rmem_default

# Output:
net.core.rmem_max = 212992    # ~208KB (default, small)
net.core.wmem_max = 212992
net.core.rmem_default = 212992
```

**Key Insight**: Default 208KB buffers work fine for LOCAL testing, but matter more for real WiFi with latency/jitter.

---

### Simulating WiFi Congestion with Traffic Control

#### Attempt 1: Too Aggressive (50 Mbps)
```bash
# This broke connectivity completely!
tc qdisc add dev lo root tbf rate 50mbit burst 32kbit latency 100ms

# Result: Zenoh discovery failed, topic not found
# Lesson: Severe congestion breaks connectivity, not just slows it
```

#### Attempt 2: Realistic WiFi (200 Mbps + 10ms jitter)
```bash
# Remove previous rule
tc qdisc del dev lo root

# Add realistic WiFi simulation
tc qdisc add dev lo root netem delay 10ms 5ms rate 200mbit

# Verify it's applied
tc qdisc show dev lo
# Output: qdisc netem ... delay 10ms 5ms rate 200Mbit
```

**Results with simulated congestion:**
```
FPS: ~5 FPS (down from 30!)
Jitter (std dev): 0.07s (up from 0.0004s!)
```

### Eureka Moment #1: 83% FPS Drop on Congested WiFi

| Condition | FPS | Jitter (std dev) |
|-----------|-----|------------------|
| No limits | 30 FPS | 0.0004s |
| 200 Mbps + 10ms delay | **5 FPS** | **0.07s** |

**Why?** Camera produces 84 MB/s = 672 Mbps, but we limited to 200 Mbps. Only ~30% of bandwidth available.

---

### Topic Bandwidth Comparison

The D435i automatically publishes multiple formats via `image_transport`:

```bash
# List all topics
ros2 topic list
```

**Available image topics:**
```
/camera/camera/color/image_raw              # Raw
/camera/camera/color/image_raw/compressed   # JPEG
/camera/camera/color/image_raw/zstd         # ZSTD
/camera/camera/depth/image_rect_raw         # Depth raw
```

**Bandwidth measurements (with 200 Mbps limit):**

```bash
ros2 topic bw /camera/camera/color/image_raw
# Result: ~12 MB/s, 2.76 MB per frame, ~5 FPS

ros2 topic bw /camera/camera/depth/image_rect_raw
# Result: ~24 MB/s, 0.81 MB per frame, ~30 FPS

ros2 topic bw /camera/camera/color/image_raw/compressed
# Result: ~7 MB/s, 0.23 MB per frame, ~30 FPS

ros2 topic bw /camera/camera/color/image_raw/zstd
# Result: ~18 MB/s, 1.45 MB per frame, ~12 FPS
```

### Eureka Moment #2: Compression is the WiFi Solution!

| Topic | Size | Bandwidth | FPS (congested) | Compression Ratio |
|-------|------|-----------|-----------------|-------------------|
| Color RAW | 2.76 MB | ~12 MB/s | ~5 FPS | 1x (baseline) |
| Color ZSTD | 1.45 MB | ~18 MB/s | ~12 FPS | 1.9x |
| **Color JPEG** | **0.23 MB** | **~7 MB/s** | **~30 FPS** | **12x smaller!** |
| Depth RAW | 0.81 MB | ~24 MB/s | ~30 FPS | N/A |

**Key Finding**: JPEG compressed topic (0.23 MB) achieves **full 30 FPS** on the same congested network where raw (2.76 MB) only gets 5 FPS!

### Eureka Moment #3: Smaller Messages Win on Congested Networks

Depth frames (0.81 MB) get through at full 30 FPS while color frames (2.76 MB) get throttled to ~5 FPS on the same congested link.

---

### Who Does the Compression?

**The `image_transport` ROS 2 package does it automatically!**

When you launch the camera, `image_transport` plugins publish multiple versions:
- `/image_raw` - Original uncompressed
- `/image_raw/compressed` - JPEG compressed
- `/image_raw/zstd` - ZSTD compressed
- `/image_raw/theora` - Video codec

**No extra configuration needed** - just subscribe to the `/compressed` topic instead of `/image_raw`.

---

### Cleanup Commands

```bash
# Remove traffic shaping when done
tc qdisc del dev lo root

# Verify removed
tc qdisc show dev lo
# Should show: qdisc noqueue 0: root refcnt 2
```

---

## Key Takeaways for Exercise 6

1. **Kernel tuning matters for real WiFi** (latency/jitter), not local testing
2. **Severe congestion breaks connectivity** (not just slows it)
3. **Compression is the real solution for WiFi**:
   - Raw 720p: 2.76 MB → 5 FPS on congested WiFi
   - JPEG compressed: 0.23 MB → 30 FPS on same WiFi
4. **Use `/compressed` topics over WiFi**, not raw images
5. **Smaller message sizes inherently perform better** on bandwidth-limited links

---

## Traffic Control Quick Reference

```bash
# Add bandwidth limit with delay
tc qdisc add dev lo root netem delay 10ms 5ms rate 200mbit

# Remove all tc rules
tc qdisc del dev lo root

# Show current rules
tc qdisc show dev lo
```

---

## Topic Filtering Demonstration (Practical)

### The Test
With traffic shaping enabled (200 Mbps + 10ms jitter), subscribe to different topics:

```bash
# Enable traffic shaping
tc qdisc add dev lo root netem delay 10ms 5ms rate 200mbit

# Test raw topic
ros2 topic hz /camera/camera/color/image_raw
# Result: ~5 FPS, std dev 0.067s

# Test compressed topic
ros2 topic hz /camera/camera/color/image_raw/compressed
# Result: ~30 FPS, std dev 0.004s

# Cleanup
tc qdisc del dev lo root
```

### Results

| Topic | FPS | Jitter (std dev) |
|-------|-----|------------------|
| `/image_raw` (2.76 MB) | **~5 FPS** | 0.067s |
| `/image_raw/compressed` (0.23 MB) | **~30 FPS** | 0.004s |

### Eureka Moment #4: Application-Level Filtering Works!

**Same network, same camera, 6x better FPS** - just by subscribing to `/compressed` instead of `/image_raw`!

**Practical Lesson**: For WiFi deployments, configure your subscribers to use compressed image transport:
- In launch files: Use `image_transport:=compressed`
- In code: Subscribe to `/camera/.../compressed` topics
- Result: Full frame rate on bandwidth-limited networks

---

## Exercise 7: Congestion Handling

### Setup
- Camera publishing at 30 FPS (large images)
- cmd_vel publishing at 10 Hz (tiny control commands)
- Traffic shaping to simulate congestion

### Baseline (No Congestion)
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.1}}" --rate 10
ros2 topic hz /cmd_vel
ros2 topic hz /camera/camera/color/image_raw
```

| Topic | Rate | Jitter (std dev) |
|-------|------|------------------|
| `/cmd_vel` | 10 Hz | 0.0003s |
| `/camera/image_raw` | 30 FPS | 0.0005s |

### Moderate Congestion (200 Mbps + 10ms delay)
```bash
tc qdisc add dev lo root netem delay 10ms 5ms rate 200mbit
ros2 topic hz /cmd_vel
ros2 topic hz /camera/camera/color/image_raw
```

| Topic | Rate | Jitter (std dev) |
|-------|------|------------------|
| `/cmd_vel` | 10 Hz | 0.0035s (**10x worse**) |
| `/camera/image_raw` | ~5 FPS | 0.04s |

### Severe Congestion (50 Mbps + 50ms delay)
```bash
tc qdisc del dev lo root
tc qdisc add dev lo root netem delay 50ms 20ms rate 50mbit
ros2 topic hz /cmd_vel
```

| Topic | Rate | Jitter (std dev) | Min/Max Interval |
|-------|------|------------------|------------------|
| `/cmd_vel` | 10 Hz | **0.022s** | 62-135ms (expected: 100ms) |

### Eureka Moment #5: Zenoh Handles Small Packets Well!

**Surprising Finding**: cmd_vel (tiny messages) maintained 10 Hz rate even under severe congestion!

Unlike classic "head-of-line blocking", small messages slip through in Zenoh/TCP.

**BUT timing consistency degrades significantly:**
- Jitter increased 73x (0.0003s → 0.022s)
- Interval variance: ±37ms instead of ±1ms

**Impact on Robots:**
- Jerky movement from inconsistent timing
- Control loop instability
- Path following errors

**Lesson**: Priority queues help ensure **consistent timing**, not just delivery. For safety-critical systems, predictable timing matters as much as throughput.

### Cleanup
```bash
tc qdisc del dev lo root
```

---

## Exercise 8: NAT Traversal & Namespace Resolution

### Two Concepts
1. **NAT Traversal** (STUN/TURN/ICE) - Theoretical, requires actual NAT environments to test
2. **Namespace Resolution** - Practical, can test locally with ROS_NAMESPACE

### Namespace Resolution Testing

#### The Problem: Topic Collisions
Without namespaces, multiple robots publishing `/chatter` would cause confusion:
```
Robot 1 publishes: /chatter
Robot 2 publishes: /chatter   ──►  Which /chatter is which?!
```

#### The Solution: Namespace Prefixes

```bash
# Robot 1: Start talker with namespace
ros2 run demo_nodes_cpp talker --ros-args --remap __ns:=/robot1

# Robot 2: Start talker with namespace (different node name to avoid collision)
ros2 run demo_nodes_cpp talker --ros-args --remap __ns:=/robot2 --remap __node:=talker2
```

**Note**: Using `--ros-args --remap __ns:=/robotX` is more reliable than `ROS_NAMESPACE` environment variable.

#### Results

```bash
# Check nodes
ros2 node list
# Output:
# /robot1/talker
# /robot2/talker2

# Check topics
ros2 topic list | grep robot
# Output:
# /robot1/chatter
# /robot2/chatter
```

#### Topic Info Shows Clear Separation

```bash
ros2 topic info /robot1/chatter -v
# Node name: talker
# Node namespace: /robot1

ros2 topic info /robot2/chatter -v
# Node name: talker2
# Node namespace: /robot2
```

#### Independent Message Streams

```bash
ros2 topic echo /robot1/chatter --once
# data: 'Hello World: 27'

ros2 topic echo /robot2/chatter --once
# data: 'Hello World: 29'
```

Both robots publish independently with their own counters!

### Eureka Moment #6: Namespace Resolution Prevents Chaos!

| Robot | Node Name | Namespace | Topic |
|-------|-----------|-----------|-------|
| Robot 1 | `talker` | `/robot1` | `/robot1/chatter` |
| Robot 2 | `talker2` | `/robot2` | `/robot2/chatter` |

**Key Findings:**
- **Namespaces prevent topic collisions** - essential for multi-robot fleets
- **Use `--ros-args --remap __ns:=/robotX`** - more reliable than environment variables
- **Fleet management pattern**: Use topic prefix patterns like `/robot*/chatter` to monitor all robots

### NAT Traversal (Conceptual)

NAT traversal is harder to test locally (requires actual NAT environments). Key concepts:

| Protocol | Purpose |
|----------|---------|
| **STUN** | Discover public IP/port ("what's my public address?") |
| **TURN** | Relay when direct P2P fails |
| **ICE** | Try STUN first, fall back to TURN |

For real NAT traversal testing, you would need:
1. Two networks behind different NATs
2. A STUN/TURN server accessible from both
3. Zenoh configured with ICE endpoints

---

## Key Takeaways for Exercise 8

1. **Namespace resolution is essential for multi-robot systems**
2. **Use `--ros-args --remap __ns:=/robotX`** for reliable namespace assignment
3. **Each robot gets isolated topics**: `/robot1/odom`, `/robot2/odom`, etc.
4. **Fleet monitoring**: Subscribe to patterns like `/robot*/status`
5. **NAT traversal requires real network topology** - difficult to simulate locally

---

## Blog Content Recommendations

### Diagrams to Include

1. **Bandwidth Comparison Bar Chart**
```
Raw Color:        84 MB/s  ████████████████████
ZSTD Compressed:  18 MB/s  ████
JPEG Compressed:   7 MB/s  ██
```

2. **Before/After Congestion Table**

3. **Traffic Control Command Reference**

### Eureka Moments Summary

| # | Discovery | What We Learned |
|---|-----------|-----------------|
| 1 | 83% FPS Drop | 200 Mbps limit drops 720p from 30 FPS to 5 FPS |
| 2 | Compression Wins | JPEG (12x smaller) achieves full 30 FPS on congested WiFi |
| 3 | Size Matters | Smaller messages (depth 0.81 MB) get through when larger ones (color 2.76 MB) don't |
| 4 | App-Level Filtering | Subscribe to `/compressed` instead of `/image_raw` = 6x better FPS on same network |
| 5 | Zenoh Handles Small Packets | cmd_vel maintains 10 Hz even under severe congestion, but jitter increases 73x |
| 6 | Namespace Resolution | Use `--ros-args --remap __ns:=/robotX` to prevent topic collisions in multi-robot fleets |

---

### Diagrams Recommended for Blog

1. **Bandwidth Comparison Bar Chart** (Exercise 6)
2. **Before/After Congestion Table** (Exercise 6)
3. **Head-of-Line Blocking Diagram** (Exercise 7) - show how small messages slip through
4. **Multi-Robot Namespace Topology** (Exercise 8) - fleet management pattern
5. **Traffic Control Command Reference Card** (Quick reference)

---

*Session notes completed: December 16, 2025*
*Exercises 6-8 all completed with hands-on testing*
