import carla
import numpy as np
import weakref
import cv2
import pygame

from carla import ColorConverter as cc

try:
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image as ROSImage
    from sensor_msgs.msg import CameraInfo
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped
    import tf2_ros
    import rclpy
    from rclpy.node import Node
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


def generate_camera_info(image_width, image_height, fov_deg, baseline=0.4, is_left=True):
    cx = image_width / 2.0
    cy = image_height / 2.0
    f = image_width / (2.0 * np.tan(fov_deg * np.pi / 360.0))
    K = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0]
    D = [0.0] * 5
    R = [1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0]
    Tx = 0.0 if is_left else -baseline * f
    P = [f, 0.0, cx, Tx, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

    info = CameraInfo()
    info.width = image_width
    info.height = image_height
    info.k = K
    info.d = D
    info.r = R
    info.p = P
    info.distortion_model = "plumb_bob"
    return info


class StereoCameraSensor:
    def __init__(
        self, parent, transform, is_left=True, gamma=2.2, hud=None,
        image_width=400, image_height=400, fov_deg=90.0, baseline=0.4,
        frame_id="camera", parent_frame="base_link", ros_node: Node = None
    ):
        self._parent = parent
        self.is_left = is_left
        self.hud = hud
        self.frame_id = f"{frame_id}_{'left' if is_left else 'right'}"
        self.parent_frame = parent_frame
        self.ros_node = ros_node

        self.bridge = CvBridge() if ROS_AVAILABLE else None
        self.cv_image = None
        self.surface = None
        self.recording = False
        self.camera_info = generate_camera_info(image_width, image_height, fov_deg, baseline, is_left=is_left)

        world = parent.get_world()
        bp = world.get_blueprint_library().find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', str(image_width))
        bp.set_attribute('image_size_y', str(image_height))
        if bp.has_attribute('gamma'):
            bp.set_attribute('gamma', str(gamma))

        self.sensor = world.spawn_actor(bp, transform, attach_to=parent,
                                        attachment_type=carla.AttachmentType.Rigid)
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda image: StereoCameraSensor._callback(weak_self, image))

        if self.hud:
            self.hud.notification(f"{'Left' if is_left else 'Right'} RGB Camera Initialized")

        # Publish static TF if ROS is available and node is passed
        if ROS_AVAILABLE and self.ros_node is not None:
            self._publish_static_tf(transform)

    def _publish_static_tf(self, transform: carla.Transform):
        broadcaster = tf2_ros.StaticTransformBroadcaster(self.ros_node)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.parent_frame
        tf_msg.child_frame_id = self.frame_id

        tf_msg.transform.translation.x = transform.location.x
        tf_msg.transform.translation.y = transform.location.y
        tf_msg.transform.translation.z = transform.location.z

        quat = transform.rotation.get_quaternion()
        tf_msg.transform.rotation.x = quat.x
        tf_msg.transform.rotation.y = quat.y
        tf_msg.transform.rotation.z = quat.z
        tf_msg.transform.rotation.w = quat.w

        broadcaster.sendTransform(tf_msg)
        self.ros_node.get_logger().info(f"Published static TF from {self.parent_frame} to {self.frame_id}")

    @staticmethod
    def _callback(weak_self, image):
        self = weak_self()
        if not self:
            return
        image.convert(cc.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
        bgr = array[:, :, :3][:, :, ::-1]
        self.cv_image = bgr

    def render(self, display, offset=(0, 0)):
        if self.surface:
            display.blit(self.surface, offset)

    def publish_ros(self, image_pub, info_pub, timestamp=None):
        if not ROS_AVAILABLE or self.cv_image is None:
            return

        img_msg = self.bridge.cv2_to_imgmsg(cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB), encoding="rgb8")
        img_msg.header.frame_id = self.frame_id
        img_msg.header.stamp = timestamp or Time()
        image_pub.publish(img_msg)

        info = self.camera_info
        info.header.frame_id = img_msg.header.frame_id
        info.header.stamp = img_msg.header.stamp
        info_pub.publish(info)

        if self.ros_node:
            self.ros_node.get_logger().info(f"Published image and CameraInfo for {self.frame_id}")

    def destroy(self):
        if self.sensor:
            self.sensor.stop()
            self.sensor.destroy()
        self.sensor = None


class StereoCameraManager:
    def __init__(self, parent, gamma=2.2, hud=None):
        self.left_cam = StereoCameraSensor(parent, carla.Transform(carla.Location(x=1.5, y=-0.2, z=2.4)),
                                           is_left=True, gamma=gamma, hud=hud)
        self.right_cam = StereoCameraSensor(parent, carla.Transform(carla.Location(x=1.5, y=0.2, z=2.4)),
                                            is_left=False, gamma=gamma, hud=hud)

    def render(self, display):
        self.left_cam.render(display, offset=(0, 0))
        self.right_cam.render(display, offset=(400, 0))

    def publish(self, left_pub, right_pub, left_info_pub, right_info_pub, timestamp=None, frame_id="camera"):
        self.left_cam.publish_ros(left_pub, left_info_pub, timestamp)
        self.right_cam.publish_ros(right_pub, right_info_pub, timestamp)

    def toggle_recording(self):
        state = not self.left_cam.recording
        self.left_cam.recording = state
        self.right_cam.recording = state

    def destroy(self):
        self.left_cam.destroy()
        self.right_cam.destroy()



# === Single Camera Manager ===
class CameraManager:
    def __init__(self, parent_actor, hud, gamma_correction):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        self.index = None
        self.lidar_range = 50
        self._bridge = CvBridge() if ROS_AVAILABLE else None

        self._camera_transforms = [
            (carla.Transform(carla.Location(x=1.5, z=2.4)), carla.AttachmentType.Rigid)
        ]
        self.transform_index = 0

        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth', {}],
        ]

        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()

        for item in self.sensors:
            bp = bp_library.find(item[0])
            if 'camera' in item[0]:
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
            item.append(bp)

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)

    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors)
        if self.index != index or force_respawn:
            if self.sensor:
                self.sensor.destroy()
                self.surface = None
            bp = self.sensors[index][-1]
            self.sensor = self._parent.get_world().spawn_actor(
                bp,
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1]
            )
            self.sensor.listen(lambda image: CameraManager._parse_image(weakref.ref(self), image))
            if notify:
                self.hud.notification(self.sensors[index][2])
            self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification(f"Recording {'On' if self.recording else 'Off'}")

    def render(self, display):
        if self.surface:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        self.surface = pygame.surfarray.make_surface(CameraManager._convert_image_to_array(self, image))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)

    @staticmethod
    def _convert_image_to_array(self, image):
        image.convert(self.sensors[self.index][1])
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))
        return array[:, :, :3][:, :, ::-1].swapaxes(0, 1)
