# Visual SLAM Pipeline with CARLA + Isaac ROS

This repository provides a structured setup for running a Visual SLAM pipeline using synchronized stereo camera data from CARLA integrated with NVIDIA Isaac ROS.

## 🚀 Overview

This project demonstrates:
- Stereo camera data streaming from CARLA
- ROS2-based image publishing
- Integration with Isaac ROS Visual SLAM
- Real-time pose estimation

## 🧱 System Components

- CARLA Simulator (UE5)
- ROS2 (Humble)
- NVIDIA Isaac ROS
- Docker-based development environment

## ⚙️ Setup

See detailed setup instructions here:
👉 [Setup Guide](setup.md)

## 🔁 Pipeline Steps

1. Launch CARLA simulator  
2. Run stereo camera publisher  
3. Launch Isaac ROS Visual SLAM  
4. Monitor pose output  

👉 Full pipeline details:
👉 [Pipeline Guide](run_pipeline.md)

## 📊 Output

- Topic: `/visual_slam/tracking/vo_pose`
- Provides real-time pose estimation

## 🧪 Notes

- Tested in Docker environment with GPU support
- Designed for experimentation with CARLA-ROS integration

## 📌 Future Improvements

- Add visualization (RViz)
- Integrate with navigation stack
- Improve synchronization robustness

## 👩‍💻 Author

Humaira Tasnim  
MS Mechanical Engineering, Tennessee Tech University
