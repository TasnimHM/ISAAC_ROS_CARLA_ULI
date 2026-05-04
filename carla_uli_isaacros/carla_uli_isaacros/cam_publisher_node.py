import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')

        # Publishers
        self.left_pub = self.create_publisher(CameraInfo, '/fixed/carla/hero/rgb_left/camera_info', 10)
        self.right_pub = self.create_publisher(CameraInfo, '/fixed/carla/hero/rgb_right/camera_info', 10)

        # Subscriptions to get the timestamps
        self.left_img_sub = self.create_subscription(Image, '/carla/hero/rgb_left/image', self.left_cb, 10)
        self.right_img_sub = self.create_subscription(Image, '/carla/hero/rgb_right/image', self.right_cb, 10)

    def left_cb(self, msg):
        info = self.make_info('rgb_left', msg.header.stamp)
        self.left_pub.publish(info)

    def right_cb(self, msg):
        info = self.make_info('rgb_right', msg.header.stamp)
        self.right_pub.publish(info)

    def make_info(self, frame_id, stamp):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = frame_id
        info.width = 400
        info.height = 400

        fx = fy = 200.0  # derived from FOV = 90
        cx = cy = 200.0  # center of image

        info.k = [fx, 0.0, cx,
                  0.0, fy, cy,
                  0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0,
                  0.0, fy, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        info.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # no distortion
        info.distortion_model = 'plumb_bob'
        return info


def main():
    rclpy.init()
    rclpy.spin(CameraInfoPublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
