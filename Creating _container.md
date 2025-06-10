# ISAAC_ROS_CARLA_ULI

This repository contains configuration and setup files for integrating **Isaac ROS** with **CARLA** for Urban-Level Intelligence (ULI) tasks such as localization, perception, and visual SLAM.

## 🚀 Getting Started

This setup assumes you are using a Docker container based on the `isaac_ros_dev-x86_64` image with GPU support and a mounted workspace directory.

### 🐳 Run the Docker Container
Create work directory:

```bash
mkdir -p ~/Works/isaac_ros_carla_uli_ws/src
cd ~/Works/isaac_ros_carla_uli_ws
```

Use the following command to launch the Isaac ROS container with GUI support and bind your local workspace:

```bash
docker run -it \
  --gpus all \
  --net=host \
  --env DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/Works/isaac_ros_carla_uli_ws:/workspaces/isaac_ros_carla_uli_ws \
  --name isaac_carla_container \
  --entrypoint bash \
  -w /workspaces/isaac_ros_carla_uli_ws \
  isaac_ros_dev-x86_64
```
