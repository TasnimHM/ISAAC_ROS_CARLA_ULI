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

### ROS2 Node Testing

Navigate to the src/ folder and create ROS2 Pyhton package:
```bash
cd /workspaces/isaac_ros_carla_uli_ws/src
ros2 pkg create --build-type ament_python stereo_cam_node --dependencies rclpy sensor_msgs
```

Add Add stereo_subscriber.py:

```bash
cd /workspaces/isaac_ros_carla_uli_ws/src/stereo_cam_node/stereo_cam_node
nano stereo_subscriber.py
```
Paste the following : 

```bash
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

class StereoCamNode(Node):
    def __init__(self):
        super().__init__('stereo_cam_node')
        self.create_subscription(Image, '/carla/left/image_raw', self.left_image_cb, 10)
        self.create_subscription(Image, '/carla/right/image_raw', self.right_image_cb, 10)
        self.create_subscription(CameraInfo, '/carla/left/camera_info', self.left_info_cb, 10)
        self.create_subscription(CameraInfo, '/carla/right/camera_info', self.right_info_cb, 10)

    def left_image_cb(self, msg):
        self.get_logger().info('📸 Left image received.')

    def right_image_cb(self, msg):
        self.get_logger().info('📸 Right image received.')

    def left_info_cb(self, msg):
        self.get_logger().info('📷 Left camera info received.')

    def right_info_cb(self, msg):
        self.get_logger().info('📷 Right camera info received.')

def main(args=None):
    rclpy.init(args=args)
    node = StereoCamNode()
    rclpy.spin(node)
    rclpy.shutdown()

```
Save and exit with Ctrl + O, Enter, then Ctrl + X.

Make it executable:

```bash
chmod +x stereo_subscriber.py
```

Edit setup.py:
Go to your package root:
```bash
cd /workspaces/isaac_ros_carla_uli_ws/src/stereo_cam_node
nano setup.py
```
Add this : 

```bash
from setuptools import find_packages, setup

package_name = 'stereo_cam_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Humaira Tasnim',
    maintainer_email='htasnim42@tntech.edu',
    description='Minimal stereo camera ROS 2 node for CARLA',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stereo_subscriber = stereo_cam_node.stereo_subscriber:main',
        ],
    },
)
```
Save and exit with Ctrl + O, Enter, then Ctrl + X.

Built the package: 

```bash
cd /workspaces/isaac_ros_carla_uli_ws
colcon build --packages-select stereo_cam_node
```

Source the Workspace:

```bash
source install/setup.bash
```
Run the node : 

```bash
ros2 run stereo_cam_node stereo_subscriber
```

Open a new terminal tab or window and get inside the container :

```bash
docker exec -it isaac_carla_container bash
```

Source the Workspace in second terminal:

```bash
source install/setup.bash
```
Check the topics :

```bash
ros2 node list
```
And you will see /stereo_cam_node if it's working

To get inside the container again after exiting:

```bash
docker start -ai isaac_carla_container
```


