# 🤖 Auto Marker Docking for Mirte Master (ROS 2 Humble)

This package enables the **Mirte Master** robot to automatically detect an **ArUco marker** using its **Orbbec Astra Pro Plus** camera and drive toward it smoothly and precisely.  
The robot aligns itself, approaches, and performs a stable final docking stop.

---

## 🔥 Key Features

| Capability | Description |
|-----------|-------------|
| 🎯 Marker tracking | Uses OpenCV ArUco detection |
| 🚗 Automatic approach | Robot drives toward marker center |
| 🎮 Smooth steering | Rotational correction only when needed |
| ⚖️ Adaptive speed | Fast when far, slow when close |
| 🛑 Precision stop | Stops at configurable distance |
| 🧪 Debug tools | Optional viewer to visualize detection |

Designed specifically for:

| Hardware | Status |
|---------|--------|
| **Mirte Master (TUDelft)** | ✅ Supported |
| **Orbbec Astra Pro Plus camera** | ✅ Supported |

---

## 📦 Requirements

- ROS 2 **Humble**
- Orbbec Astra Pro Plus streaming RGB (default Mirte Master camera, change camera topic if using other camera)

### System Dependencies

```bash
sudo apt install ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-vision-opencv libeigen3-dev
````

---

## 🔧 Installation

```bash
# Move to workspace
cd ~/mirte_ws/src

# Clone package
git clone https://github.com/deniseheidema/Mirte-Master-Auto-Docking-ArUco.git

# Build
cd ~/mirte_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

---

## 🚀 Usage

### 1. Start Autonomous Docking

```bash
ros2 launch auto_marker_docking auto_marker_docking.launch.py
```

---

## 🎛 Parameter Configuration

Edit parameters:

```bash
nano ~/mirte_ws/src/auto_marker_docking/config/params.yaml
```

| Parameter            | Description                   | Default                          |
| -------------------- | ----------------------------- | -------------------------------- |
| `camera_topic`       | RGB camera topic              | `/camera/color/image_raw`        |
| `cmd_vel_topic`      | Motor control output topic    | `/mirte_base_controller/cmd_vel` |
| `marker_size`        | Marker size in meters         | `0.10`                           |
| `stop_distance`      | Distance from marker to stop  | `0.25`                           |
| `drive_speed`        | Base forward speed            | `0.3`                            |
| `yaw_gain`           | Steering correction gain      | `0.8`                            |
| `yaw_turn_threshold` | Angle before turning in-place | `0.09` (~5°)                     |

---

## 🧪 Debug Marker Viewer

```bash
ros2 run auto_marker_docking aruco_simple_detector
```

Shows:

* Marker ID
* Pose estimation
* Live annotated camera video

---

## 🖨 Print ArUco Markers

Generate markers here:
[https://chev.me/arucogen/](https://chev.me/arucogen/)

| Setting               | Value                              |
| --------------------- | ---------------------------------- |
| Dictionary            | `6x6_250`                          |
| Printed size          | Must match `marker_size` parameter |

---

## 🧯 Troubleshooting

| Issue                          | Cause                     | Fix                                                            |
| ------------------------------ | ------------------------- | -------------------------------------------------------------- |
| Robot wobbles / oscillates     | Steering gain too high    | Lower `yaw_gain`, increase `yaw_turn_threshold`                |
| Marker not detected            | Wrong topic               | Check with `ros2 topic list`                                   |
| Robot turns the wrong way      | Yaw is inverted           | Change `last_marker_yaw_ = -yaw;` to `last_marker_yaw_ = yaw;` |
| Changes in code not working    | Code has not been rebuild | Rebuild code with `colcon build --symlink-install`

---

## 🤝 Contributing

Pull requests and improvements are welcome!

---

## 📜 License

MIT License — free to use and modify.
