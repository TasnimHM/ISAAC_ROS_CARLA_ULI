
# Visual SLAM Pipeline with Synchronized Stereo Images from CARLA

To run the Visual SLAM pipeline with synchronized stereo images from CARLA, we followed a structured multi-step process:

---

## ✅ Setup and Verification Steps

1. **Validate Stereo Camera Publisher:**
   - Verified that the stereo camera node correctly published images and camera info using a custom `ros2_native.py` script.

2. **Avoid Unnecessary ROS 2 Folder Changes:**
   - Ensured synchronization between CARLA and Isaac ROS without moving or overwriting the `ros2/` folder structure.

3. **Fix Sensor Configuration:**
   - Resolved minor config issues by removing unsupported attributes (e.g., `film_grain_intensity`) that caused rendering issues.

4. **Confirm VO Pose Output:**
   - Confirmed that `/visual_slam/tracking/vo_pose` was being successfully published by the SLAM node.

---

## ✅ Final Working Command Sequence

### 1. Start CARLA (from UE5 Docker setup or host machine):
```bash
docker-compose up
```

### 2. Run the CARLA Stereo ROS Publisher:
```bash
cd /workspaces/isaac_ros-dev/temp/examples/ros2
python3 ros2_native.py --file stack.json
```

### 3. Launch the Isaac ROS Visual SLAM Pipeline:
```bash
source install/setup.bash
ros2 launch carla_uli_isaacros isaac_ros_visual_slam_carla_sim.launch.py
```

### 4. Verify SLAM Output Pose:
```bash
ros2 topic echo /visual_slam/tracking/vo_pose
```

---

