# Workshop 4 Research Notes: IMU-Centric Perception in ROS 2

## About Yaanendriya Pvt. Ltd.

### Company Profile
- **Founded:** 2023 in Bengaluru, Karnataka
- **Incubation:** ARTPARK (AI & Robotics Technology Park) at IISc Bangalore
- **Focus:** Indigenizing India's sensor ecosystem for autonomous systems
- **Support:** Ministry of Heavy Industries, Department of Science & Technology
- **Mission:** Building India's indigenous sensor ecosystem for autonomous systems

### ARTPARK Connection
- ARTPARK is the AI & Robotics Technology Park established at IISc Bangalore
- Focuses on translating deep-tech research into commercial products
- Yaanendriya is one of their incubated startups

### Their Products (Relevant to Workshop)

| Product | Description | Relevance |
|---------|-------------|-----------|
| **yDx.M** | High-precision Inertial Sensing Module (AHRS) | **Exercise 3** will use this |
| | 6/9/10 DOF configurations | Primary motion source |
| | ±2/4/8/16G accelerometer, 450-2000°/sec gyro | Distributed sensor network |
| **Syncore Nano** | Navigation stack controller | Drone/ground robot control |
| **YPS M9N** | GNSS module (u-blox M9) | Positioning |

### yDx.M Specifications
- **Configurations:** 6-DOF, 9-DOF, 10-DOF
- **Accelerometer:** ±2/4/8/16G configurable
- **Gyroscope:** 450-2000°/sec configurable
- **Magnetometer:** Available in 9/10 DOF versions
- **Factory Calibrated:** Yes (unlike D435i)
- **Distributed Sensing:** Network of sensors possible
- **Interface:** I2C/UART/SPI

### No Public Content Found
- No published workshops, tutorials, or GitHub repositories
- This appears to be their first major public workshop
- You'll be learning directly from the product creators!

---

## Intel RealSense D435i IMU Specifications

### Hardware
| Component | Spec | Notes |
|-----------|------|-------|
| **IMU Chip** | Bosch BMI055 | 6-DOF (no magnetometer) |
| **Accelerometer** | 12-bit, ±2/4/8/16G | Linear acceleration in m/s² |
| **Gyroscope** | 16-bit, ±2000°/s | Angular velocity in rad/sec |
| **Output Rate** | Up to 400 Hz | Hardware synchronized |

### Critical Limitations
- **IMU is NOT pre-calibrated** - Must run calibration tool first!
- **No magnetometer** - Cannot get absolute heading, only relative orientation
- **Yaw drift is unbounded** - Without magnetometer, yaw drifts continuously

### Why Perfect for Workshop
- Experience the PROBLEMS that yDx.M solves:
  1. Need for calibration (yDx.M is factory calibrated)
  2. No absolute heading (yDx.M has magnetometer options)
  3. Single sensor point of failure (yDx.M supports distributed sensing)

---

## Docker Container: workshop4-imu

### Pre-installed Packages
```
ros-humble-imu-tools              # Exercise 1-2: Madgwick/complementary filters
ros-humble-robot-localization     # Exercise 3: Sensor fusion with EKF/UKF
ros-humble-rtabmap-ros            # Exercise 4: Inertial-aided SLAM
ros-humble-ros2bag               # Exercise 5: Record/replay
Intel RealSense SDK + ROS2        # D435i camera + IMU driver
Isaac ROS Visual SLAM             # GPU-accelerated VSLAM
```

### Container Details
- **Image:** workshop4-imu:latest (22.4GB)
- **Launch:** `./scripts/launch-container.sh 4`
- **ROS_DOMAIN_ID:** 40
- **RMW:** CycloneDDS
- **GPU:** NVIDIA passthrough enabled
- **Devices:** USB/HID access for RealSense

---

## ROS 2 IMU Ecosystem

### Key Packages

#### 1. imu_filter_madgwick
- **Purpose:** Compute orientation from raw IMU data
- **Algorithm:** Madgwick AHRS filter
- **Input:** `sensor_msgs/Imu` (raw - no orientation)
- **Output:** `sensor_msgs/Imu` (with orientation quaternion)
- **Parameters:**
  - `use_mag`: false for D435i (no magnetometer)
  - `world_frame`: "enu" for ROS standard
  - `publish_tf`: true for visualization

#### 2. robot_localization
- **Purpose:** Multi-sensor fusion with EKF/UKF
- **Nodes:** ekf_node, ukf_node, navsat_transform_node
- **Fusion:** Can combine IMU, odometry, GPS, visual odometry
- **Config:** YAML file specifies which sensor measurements to fuse
- **Critical:** Requires ENU coordinate frame

#### 3. rtabmap_ros
- **Purpose:** Visual SLAM with IMU integration
- **Features:** Loop closure, graph optimization, 3D mapping
- **IMU Integration:** Pre-integration for fast motion handling
- **Launch parameter:** `imu_topic:=/imu/data wait_imu_to_init:=true`

#### 4. Isaac ROS Visual SLAM
- **Purpose:** GPU-accelerated VSLAM
- **Features:** Real-time stereo visual odometry
- **IMU Fusion:** `enable_imu_fusion:=true`
- **Performance:** Leverages NVIDIA GPU

---

## Key Concepts

### 1. Orientation Representations
| Format | Description | Use Case |
|--------|-------------|----------|
| Quaternion | (w, x, y, z) - 4 values | ROS standard, no gimbal lock |
| Euler (RPY) | Roll, Pitch, Yaw angles | Human readable, has gimbal lock |
| Rotation Matrix | 3x3 matrix | Composing rotations |

### 2. Coordinate Frames
| Frame | Convention | Notes |
|-------|------------|-------|
| **ENU** | East-North-Up | ROS standard (REP-103) |
| **NED** | North-East-Down | Aviation/aerospace standard |
| **robot_localization** | Requires ENU | Won't work with NED! |

### 3. Filter Algorithms
| Filter | Complexity | Best For |
|--------|------------|----------|
| Complementary | Simple | Basic orientation |
| Madgwick | Medium | Real-time orientation |
| Kalman (EKF/UKF) | Complex | Multi-sensor fusion |

### 4. Error Sources in IMUs
| Error | Description | Mitigation |
|-------|-------------|------------|
| Bias drift | Constant offset accumulates | Calibration, fusion |
| Scale factor | Non-linear response | Factory calibration |
| Noise | Random fluctuations | Filtering |
| Temperature | Readings change with temp | Compensation |

---

## REP-145: IMU Sensor Driver Conventions

### Message Structure: sensor_msgs/Imu
```
std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
```

### Key Rules
1. If orientation is not known, set first element of orientation_covariance to -1
2. All values should be in SI units (rad/s, m/s²)
3. Covariance matrices are 3x3, stored row-major as 9-element array
4. Frame conventions follow REP-103 (ENU preferred)

---

## D435i vs yDx.M Comparison

| Feature | D435i IMU | yDx.M |
|---------|-----------|-------|
| DOF | 6 only | 6/9/10 configurable |
| Magnetometer | No | Optional (9/10 DOF) |
| Calibration | Manual required | Factory calibrated |
| Distributed sensing | No | Yes (network of sensors) |
| Interface | USB (via RealSense) | I2C/UART/SPI |
| Vision | Integrated RGB-D | External |
| Cost | ~$250 (camera + IMU) | TBD |

### Key Insight
We can build the same VIO solution with D435i that yDx.M + external camera provides:
- D435i IMU = yDx.M IMU (minus magnetometer, calibration)
- D435i RGB-D = External camera/LiDAR
- robot_localization EKF = yDx.M built-in fusion
- Our DIY VIO = Their production-ready solution

---

## Workshop Exercise Coverage with D435i

| Exercise | Workshop Topic | D435i Coverage | Notes |
|----------|---------------|----------------|-------|
| **1** | IMU Data Acquisition | ✅ 100% | Raw data, TF frames |
| **2** | IMU Filtering | ✅ 100% | Madgwick, complementary |
| **3** | yDx.M Fusion | ⚡ 80% | EKF fusion works, no yDx.M hardware |
| **4** | Inertial-Aided SLAM | ✅ 100% | RTAB-Map with IMU |
| **5** | End-to-End Pipeline | ✅ 100% | rosbag, debugging |

---

## Problem → Solution Journey

### Stage 1: IMU Only Problems
| Problem | How to Experience | Solution |
|---------|-------------------|----------|
| No orientation | Raw IMU has identity quaternion | Madgwick filter |
| Yaw drift | Rotate 360°, yaw doesn't return | Magnetometer (yDx.M) |
| Position drift | Integrate acceleration | Sensor fusion |
| Noise | Stationary IMU variance | Calibration + filtering |

### Stage 2: Vision Only Problems
| Problem | How to Experience | Solution |
|---------|-------------------|----------|
| Fast motion blur | Shake camera rapidly | IMU pre-integration |
| Textureless surfaces | Point at white wall | IMU dead-reckoning |
| Scale ambiguity | Monocular estimation | Depth camera or IMU |

### Stage 3: Fusion Solutions
| Problem Solved | How Fusion Helps |
|----------------|------------------|
| IMU drift | Visual odometry corrects periodically |
| Vision blur | IMU provides motion estimate during blur |
| Textureless | IMU bridges feature gaps |
| Fast motion | IMU handles high dynamics |

---

## Recommended Pre-Reading

### Official Documentation
1. [REP-145: IMU Sensor Driver Conventions](https://www.ros.org/reps/rep-0145.html)
2. [robot_localization Wiki](http://docs.ros.org/en/melodic/api/robot_localization/html/)
3. [imu_filter_madgwick ROS Index](https://index.ros.org/p/imu_filter_madgwick/)

### Tutorials
1. [Sensor Fusion with robot_localization - Automatic Addison](https://automaticaddison.com/sensor-fusion-using-the-robot-localization-package-ros-2/)
2. [Nav2 VIO Integration](https://docs.nav2.org/tutorials/docs/integrating_vio.html)

### D435i Specific
1. [Intel RealSense D435i IMU Guide](https://www.intelrealsense.com/how-to-getting-imu-data-from-d435i-and-t265/)
2. [D435i IMU Calibration PDF](https://www.intelrealsense.com/wp-content/uploads/2019/07/Intel_RealSense_Depth_D435i_IMU_Calibration.pdf)

---

## Questions to Ask at Workshop

1. **About yDx.M:** What ROS 2 driver package will we use? Is it open-source?
2. **About distributed sensing:** How does yDx.M's sensor network feature work in ROS 2?
3. **About calibration:** Does yDx.M come factory-calibrated? What calibration tools are available?
4. **About SLAM integration:** Which SLAM package does Yaanendriya recommend with yDx.M?
5. **About purchase:** How can workshop attendees obtain yDx.M modules?
