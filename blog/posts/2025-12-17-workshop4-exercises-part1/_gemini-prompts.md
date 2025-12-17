# Gemini Image Prompts for Workshop 4 Blog Series

## Part 1: IMU Alone - Watching It Drift Away

### Thumbnail (thumbnail.png)
```
Create a dramatic technical illustration showing "IMU Drift: The Silent Problem"

Show a robot/sensor with an IMU chip, and visualize the drift problem:
- A coordinate axis system that's slowly rotating/shifting
- Ghost images showing the accumulated error over time
- Numbers floating away: "0.01 m/s² → 18 meters drift!"
- Red warning indicators

Include:
- A stylized IMU sensor chip (labeled "6-DOF")
- Arrows showing acceleration and rotation measurements
- A "drift trail" showing position error accumulating
- Text overlay: "Why IMU Alone Fails"

Style: Modern technical diagram with dark blue/teal background, glowing neon effects like robotics documentation. Professional yet dramatic.
```

### Raw IMU No Orientation (raw-imu-no-orientation.png)
```
Create an infographic showing "Raw IMU Output: The Orientation Mystery"

Split view showing:
LEFT SIDE - "What Sensor Outputs":
- acceleration arrows (ax, ay, az)
- rotation arrows (ωx, ωy, ωz)
- A confused-looking robot

RIGHT SIDE - "What We Need":
- Roll, Pitch, Yaw angles
- A 3D orientation quaternion
- Question mark: "Where does this come from?"

Include the key message:
"Raw IMU = Rates, Not States!"
"Accelerometer → Force (including gravity)"
"Gyroscope → Rotation speed"
"Orientation → Computed by filter!"

Style: Technical diagram, educational, dark theme with accent colors (orange for problems, green for solutions)
```

### Yaw Drift Visualization (yaw-drift-problem.png)
```
Create a visualization of "Yaw Drift: The Magnetometer Problem"

Show a top-down view of a robot rotating 360°:
- Starting position: 0° (green)
- After 360° rotation: Should be 0°, but shows 15° (red)
- Dotted line showing the drift accumulation

Include three panels:
1. "Roll/Pitch: STABLE" (uses gravity reference) - green checkmark
2. "Yaw: DRIFTS" (no horizontal reference) - red X
3. "Solution: Magnetometer" (compass icon pointing North) - blue

Key text:
"6-DOF IMU: Yaw has NO absolute reference!"
"9-DOF IMU: Magnetometer provides North"

Style: Clean technical diagram, bird's eye view perspective, robotics aesthetic
```

### Dead Reckoning Disaster (dead-reckoning-drift.png)
```
Create an infographic showing "Dead Reckoning: Integration Disaster"

Show a timeline with position error growth:
- t=0: Robot at origin (green dot)
- t=10s: Position error 0.5m (yellow zone)
- t=30s: Position error 4.5m (orange zone)
- t=60s: Position error 18m (red zone, robot way off screen!)

Include the math:
"Error ∝ t² (quadratic growth!)"
"0.01 m/s² bias × 60s² / 2 = 18 meters"

Visual element: A path that starts straight but curves wildly off course

Key message:
"Double Integration Amplifies Every Error!"
"IMU alone CANNOT give stable position"

Style: Dramatic visualization showing the divergence, dark background with glowing error zones
```

---

## Part 2: Vision Alone - When the Camera Goes Blind

### Thumbnail (thumbnail.png)
```
Create a dramatic illustration showing "When Vision Fails"

Show a robot camera with visual problems:
- Motion blur streaks across the frame
- A featureless white wall with "0 features detected" warning
- Split scene: left side has good features (green markers), right side is blank (red warning)

Include:
- A camera lens with visual processing indicators
- Feature points scattered (some matched, some lost)
- Warning text: "Tracking Lost!"

Style: Technical diagram showing the failure modes, dark blue background, red accent for problems
```

### Fast Motion Blur (fast-motion-blur.png)
```
Create an infographic showing "Fast Motion = Lost Tracking"

Three-panel comparison:
1. SLOW MOTION (green): Clear frame with feature points, arrows showing matching
2. FAST MOTION (red): Blurred frame, features smeared, "No matches!" warning
3. IMU SOLUTION (blue): "400 Hz sampling beats motion blur"

Include timing info:
- Camera: 30 FPS = 33ms between frames
- IMU: 400 Hz = 2.5ms between samples

Key visual: Feature points becoming streaks during fast motion

Style: Technical comparison diagram, clean panels, robotics documentation style
```

### Textureless Problem (textureless-failure.png)
```
Create an infographic showing "No Features = No Tracking"

Two-panel comparison:
LEFT - "Feature-Rich Scene":
- Bookshelf, posters, objects
- Green feature markers everywhere
- "150+ features detected" ✓

RIGHT - "Textureless Scene":
- Plain white wall
- Almost no markers
- "3 features detected" ✗
- "TRACKING LOST" warning

Include common failure scenarios:
- White walls/ceilings
- Uniform floors
- Blue sky (outdoor)
- Warehouse corridors

Style: Split-screen comparison, dramatic contrast between success and failure
```

### Opposite Weaknesses (opposite-weaknesses.png)
```
Create an infographic showing "IMU vs Vision: Opposite Weaknesses"

Create a 2x4 grid comparing scenarios:

              IMU          VISION
Fast Motion   ✓ (green)    ✗ (red)
Textureless   ✓ (green)    ✗ (red)
Long Duration ✗ (red)      ✓ (green)
Lighting      ✓ (green)    ✗ (red)

Add visual icons for each scenario

Key insight at bottom:
"COMPLEMENTARY FAILURES = FUSION OPPORTUNITY!"

Arrow pointing to "VIO" (Visual-Inertial Odometry) as the solution

Style: Clean matrix visualization, color-coded for quick understanding
```

---

## Part 3: Fusion - The Best of Both Worlds

### Thumbnail (thumbnail.png)
```
Create a triumphant illustration showing "Visual-Inertial Odometry: The Solution"

Show the fusion concept:
- IMU sensor (high frequency waves)
- Camera (frames)
- Both feeding into a "FUSION" processor
- Output: Stable, robust pose estimation

Include success indicators:
- "Fast Motion: ✓"
- "Textureless: ✓"
- "Long Duration: ✓"

Visual: A robot confidently navigating with both sensor streams merging

Style: Triumphant, solution-focused, blue/green color scheme for success
```

### IMU Preintegration (imu-preintegration.png)
```
Create a timeline diagram showing "IMU Pre-Integration"

Show a timeline with:
- Vision frames at 30 Hz (vertical bars, some "blur" markers)
- IMU samples at 400 Hz (dense vertical lines)
- Between blurred vision frames: IMU samples continue
- Arrow showing "Pre-integrate IMU between frames"

Key concept:
"Even when vision fails, IMU knows how we moved!"

Include the formula concept:
"Δposition + Δvelocity + Δorientation from IMU"

Style: Technical timeline visualization, clear temporal relationship
```

### Drift Correction (drift-correction.png)
```
Create a comparison showing "How Vision Corrects IMU Drift"

Two trajectory plots:
1. IMU ONLY: Curved path drifting away (red)
   - Label: "18m drift in 60s"

2. VIO FUSION: Periodic corrections keeping path straight (green)
   - Small oscillations but bounded
   - Label: "~0.03m drift in 60s"
   - Arrows showing "vision correction" points

Key numbers:
"IMU: Unbounded drift ✗"
"VIO: Bounded drift ✓"
"600x improvement!"

Style: Graph/plot style, clear contrast between failure and success cases
```

### Complete VIO Pipeline (vio-pipeline.png)
```
Create an architecture diagram showing "Complete D435i VIO Pipeline"

Flow diagram:
┌──────────────┐
│ Intel D435i  │
├──────────────┤
│ RGB │ Depth │ IMU │
└─┬───┴───┬───┴──┬──┘
  │       │      │
  │       │      ▼
  │       │   Madgwick
  │       │   Filter
  │       │      │
  ▼       ▼      ▼
┌─────────────────────┐
│     RTAB-Map        │
│ (Visual + Inertial) │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│   Robust VIO Pose   │
└─────────────────────┘

Style: Clean architecture diagram, box-and-arrow style, professional documentation aesthetic
```

---

## Part 4: Understanding yDx.M

### Thumbnail (thumbnail.png)
```
Create an illustration showing "From DIY to Production: yDx.M"

Split comparison:
LEFT - "DIY Solution (D435i)":
- Multiple components
- Wires connecting things
- "Manual calibration" label
- "Learning setup"

RIGHT - "Production Solution (yDx.M)":
- Single integrated module
- Clean, professional
- "Factory calibrated" label
- "Deploy immediately"

Include Yaanendriya branding element and "Made in India" indicator

Bridge in middle: "Understanding → Appreciation"

Style: Product comparison style, professional, clean design
```

### yDx.M Architecture (ydxm-architecture.png)
```
Create a block diagram showing "yDx.M Internal Architecture"

Show the module internals:
┌─────────────────────────────────────┐
│            yDx.M Module             │
├─────────────────────────────────────┤
│ ┌─────────┐ ┌────────┐ ┌─────────┐ │
│ │ Accel   │ │ Gyro   │ │ Mag*    │ │
│ │ ±16G    │ │ 2000°/s│ │ 3-axis  │ │
│ └────┬────┘ └───┬────┘ └────┬────┘ │
│      │          │           │      │
│      └──────────┼───────────┘      │
│                 ▼                   │
│    ┌──────────────────────┐        │
│    │ Onboard Fusion MCU   │        │
│    │ + Factory Calibration│        │
│    └──────────┬───────────┘        │
│               │                     │
├───────────────┼─────────────────────┤
│         Output: Calibrated          │
│    Orientation + IMU Data           │
│    (I2C / UART / SPI)               │
└─────────────────────────────────────┘

*9/10-DOF versions

Style: Clean technical block diagram, professional product documentation
```

### Distributed Sensing (distributed-sensing.png)
```
Create an infographic showing "yDx.M Distributed Sensor Network"

Show a robot with multiple yDx.M modules:
- One on main body
- One on arm
- One on head/sensor mast

Compare to single IMU:
SINGLE IMU: "Single point of failure" (red X)
DISTRIBUTED: "Redundancy + accuracy" (green checkmarks)

Benefits listed:
- Fault tolerance
- Improved accuracy (averaging)
- Vibration rejection
- Flexible placement

Style: Robot visualization with sensor network overlay, professional robotics documentation
```

### DIY vs Production Comparison (comparison-table.png)
```
Create a detailed comparison infographic:

"D435i DIY vs yDx.M Production"

Feature comparison table:
                      D435i DIY    yDx.M Production
Orientation Output    Need Filter  ✓ Built-in
Magnetometer         ✗            ✓ (9/10-DOF)
Factory Calibration  ✗ Manual     ✓ Factory
Distributed Sensing  ✗            ✓
Temperature Comp     ✗            ✓
Form Factor          USB Module   I2C/UART/SPI

Use Case guidance:
D435i: "Learning, Prototyping, VIO Development"
yDx.M: "Production Robots, Drones, Safety-Critical"

Style: Clean comparison table, professional, easy to scan
```

---

## Usage Instructions

1. Copy each prompt to Google Gemini (or your preferred AI image generator)
2. Generate the image
3. Download and rename according to the filename in parentheses
4. Place in the appropriate blog post directory:
   - Part 1: `2025-12-17-workshop4-exercises-part1/`
   - Part 2: `2025-12-17-workshop4-exercises-part2/`
   - Part 3: `2025-12-17-workshop4-exercises-part3/`
   - Part 4: `2025-12-17-workshop4-exercises-part4/`

## Priority Order

Generate in this order for maximum impact:
1. **Thumbnails** (all 4) - Most visible in blog listings
2. **Part 1 images** - Foundation concepts
3. **Part 3 images** - Solution concepts
4. **Part 2 and 4 images** - Supporting content
