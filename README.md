# Visual SLAM Pipeline with CARLA + Isaac ROS

## 🚀 Overview

This repository contains exploratory work on integrating CARLA with NVIDIA Isaac ROS for visual SLAM using synchronized stereo camera data.

The goal of this project is to understand and prototype a perception pipeline involving:
- Stereo image streaming from CARLA
- ROS2-based data publishing
- Integration attempts with Isaac ROS Visual SLAM

⚠️ This is an **experimental project**, not a fully validated SLAM system.

---

## 🧱 System Components

This setup relies on the following external tools:

- CARLA Simulator (UE4/UE5)
- ROS2 (Humble)
- NVIDIA Isaac ROS (Visual SLAM)
- Docker-based development environment

👉 These components must be installed separately on your system.

---

## ⚙️ Prerequisites

Before running this project, ensure you have:

- CARLA simulator installed and running
- NVIDIA Isaac ROS Visual SLAM properly set up
- ROS2 (Humble) installed
- Docker with GPU support configured

This repository does not include these dependencies.

---

## ⚙️ Setup

See detailed setup instructions here:  
👉 [Setup Guide](setup.md)

---

## 🔁 Pipeline Steps

The following steps outline the intended pipeline:

1. Launch CARLA simulator  
2. Run stereo camera publisher  
3. Attempt to launch Isaac ROS Visual SLAM  
4. Monitor pose output (if available)

👉 Full pipeline details:  
👉 [Pipeline Guide](run_pipeline.md)

---

## 📊 Output

- Topic: `/visual_slam/tracking/vo_pose`

⚠️ Output availability may vary depending on system configuration and synchronization quality.

---

## 📝 Notes

- This project focuses on system integration and experimentation  
- Some components may require additional tuning to function reliably  
- Results may vary depending on system setup  

---

## 📌 Future Improvements

- Validate full Visual SLAM pipeline stability  
- Improve sensor synchronization between CARLA and ROS2  
- Add visualization using RViz  
- Integrate with higher-level navigation or control stack  

---

## 👩‍💻 Author

Humaira Tasnim  
MS Mechanical Engineering, Tennessee Technological University
