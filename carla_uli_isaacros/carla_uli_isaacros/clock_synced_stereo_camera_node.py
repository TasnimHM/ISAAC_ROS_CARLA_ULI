import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from rosgraph_msgs.msg import Clock
from copy import deepcopy
from cv_bridge import CvBridge
import cv2


class ClockSyncedStereoCameraNode(Node):
    def __init__(self):
        super().__init__('clock_synced_stereo_camera_node')

        self.cameras = ['rgb_left', 'rgb_right']
        self.latest_clock = None
        self.bridge = CvBridge()

        # Publishers stored per camera
        self.image_pubs = {}
        self.info_pubs = {}

        # Subscribe to simulation clock
        self.create_subscription(Clock, '/clock', self.clock_callback, 10)

        for cam in self.cameras:
            image_topic = f'/carla/hero/{cam}/image'
            image_synced_topic = f'/carla/hero/{cam}/image_synced'
            info_topic = f'/carla/hero/{cam}/camera_info_fixed_synced'

            # Image input
            self.create_subscription(Image, image_topic, lambda msg, c=cam: self.image_callback(msg, c), 10)

            # Synced image and camera info output
            self.image_pubs[cam] = self.create_publisher(Image, image_synced_topic, 10)
            self.info_pubs[cam] = self.create_publisher(CameraInfo, info_topic, 10)

    def clock_callback(self, msg: Clock):
        self.latest_clock = msg.clock

    def image_callback(self, msg: Image, cam: str):
        if not self.latest_clock:
            self.get_logger().warn(f"No clock received yet; skipping image for {cam}")
            return

        # Set timestamp to latest clock time
        synced_stamp = self.latest_clock

        # Publish camera info
        info_msg = self.get_default_camera_info(cam, synced_stamp)
        self.info_pubs[cam].publish(info_msg)

        # Convert image encoding to rgb8
        try:
            if msg.encoding == 'bgra8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgra8')
                rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2RGB)
                synced_image = self.bridge.cv2_to_imgmsg(rgb_image, encoding='rgb8')
                synced_image.header = deepcopy(msg.header)
            else:
                synced_image = deepcopy(msg)
                synced_image.encoding = 'rgb8'  # enforce compatibility

            synced_image.header.stamp = synced_stamp
            synced_image.header.frame_id = cam
            self.image_pubs[cam].publish(synced_image)

        except Exception as e:
            self.get_logger().error(f"Failed to convert image for {cam}: {e}")

    def get_default_camera_info(self, frame_id, stamp):
        info = CameraInfo()
        info.header.frame_id = frame_id
        info.header.stamp = stamp
        info.width = 600
        info.height = 600

        fx = fy = 300.0
        cx = cy = 300.0

        info.k = [fx, 0.0, cx,
                  0.0, fy, cy,
                  0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0,
                  0.0, fy, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        info.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.distortion_model = 'plumb_bob'
        return info


def main(args=None):
    rclpy.init(args=args)
    node = ClockSyncedStereoCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
