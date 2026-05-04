import rclpy
from rclpy.node import Node
import argparse
import pygame
import carla
import time

from carla_uli_isaacros.hud import HUD
from carla_uli_isaacros.world import World
from carla_uli_isaacros.keyboard_control import KeyboardControl
from sensor_msgs.msg import Image, CameraInfo


class CarlaWorldNode(Node):
    def __init__(self):
        super().__init__('carla_world_node')
        self.get_logger().info('Initializing CARLA + Pygame control loop...')

        # === Simulation parameters (replace with ROS 2 params if needed) ===
        self.args = self._parse_default_args()
        self.args.width, self.args.height = map(int, self.args.res.split('x'))

        # === Initialize CARLA client and world ===
        self.client = carla.Client(self.args.host, self.args.port)
        self.client.set_timeout(10.0)
        self.sim_world = self.client.get_world()
        self.traffic_manager = self.client.get_trafficmanager()

        if self.args.sync:
            self._enable_synchronous_mode()

        # === Initialize Pygame ===
        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode(
            (self.args.width * 2, self.args.height * 2),
            pygame.HWSURFACE | pygame.DOUBLEBUF
        )

        # === Set up HUD, world, and keyboard controller ===
        self.hud = HUD(self.args.width, self.args.height)
        self.world = World(self.sim_world, self.hud, self.traffic_manager, self.args)
        self.controller = KeyboardControl(self.world, self.args.autopilot)
        self.clock = pygame.time.Clock()

        # === ROS 2 publishers for stereo image and camera info ===
        self.left_pub = self.create_publisher(Image, "/carla/left/image_raw", 10)
        self.right_pub = self.create_publisher(Image, "/carla/right/image_raw", 10)
        self.left_info_pub = self.create_publisher(CameraInfo, "/carla/left/camera_info", 10)
        self.right_info_pub = self.create_publisher(CameraInfo, "/carla/right/camera_info", 10)

        # === Start control/render loop with 50ms period (~20 Hz) ===
        self.timer = self.create_timer(0.05, self._tick)

    def _parse_default_args(self):
        return argparse.Namespace(
            host='localhost',
            port=2000,
            res='1000x500',
            filter='vehicle.*',
            generation='All',
            rolename='hero',
            gamma=1.0,
            autopilot=False,
            sync=True
        )

    def _enable_synchronous_mode(self):
        settings = self.sim_world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        self.sim_world.apply_settings(settings)
        self.traffic_manager.set_synchronous_mode(True)

    def _tick(self):
        start_time = time.time()

        # Synchronize with simulator
        if self.args.sync:
            self.sim_world.tick()
        else:
            self.sim_world.wait_for_tick()
        after_tick = time.time()

        # Maintain loop rate
        self.clock.tick_busy_loop(60)
        after_clock = time.time()

        # Handle keyboard input
        if self.controller.parse_events(self.client, self.world, self.clock, self.args.sync):
            self.get_logger().info("Exit requested from keyboard.")
            rclpy.shutdown()
            return
        after_events = time.time()

        # Tick and render world
        self.world.tick(self.clock)
        after_world_tick = time.time()
        self.world.render(self.display)
        pygame.display.flip()
        after_render = time.time()

        # Publish stereo image + camera info to ROS 2
        self.world.stereocam.publish(
            self.left_pub, self.right_pub,
            self.left_info_pub, self.right_info_pub
        )

        # Log timing breakdown
        self.get_logger().info(
            f"Tick time: {(after_render - start_time) * 1000:.1f} ms | "
            f"SimTick: {(after_tick - start_time) * 1000:.1f} ms | "
            f"Clock: {(after_clock - after_tick) * 1000:.1f} ms | "
            f"Events: {(after_events - after_clock) * 1000:.1f} ms | "
            f"WorldTick: {(after_world_tick - after_events) * 1000:.1f} ms | "
            f"Render: {(after_render - after_world_tick) * 1000:.1f} ms"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CarlaWorldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down CarlaWorldNode...")
    finally:
        node.destroy_node()
        pygame.quit()
        rclpy.shutdown()
