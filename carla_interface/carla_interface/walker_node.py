"""
CARLA ROS 2 Integration with Pygame Manual Control and Stereo Camera Publishing

This script launches a ROS 2 node that connects to a CARLA simulation, spawns a vehicle or pedestrian
with sensors, renders the environment using Pygame, and publishes stereo images and camera info to
ROS 2 topics.

Classes:
    - World: Manages CARLA actors, sensors, weather, and rendering.
    - CarlaWorldNode: ROS 2 node that integrates CARLA with Pygame and publishes sensor data.

Topics Published:
    - /carla/left/image_raw
    - /carla/right/image_raw
    - /carla/left/camera_info
    - /carla/right/camera_info
"""

import sys
import time
import random
import argparse

import pygame
import carla
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo

# Local project modules (assuming these paths are correct in your project structure)
from carla_interface.utils.hud import HUD
from carla_interface.utils.keyboard_control import KeyboardControl
from carla_interface.utils.util import find_weather_presets, get_actor_display_name, get_actor_blueprints
from carla_interface.sensors.collision import CollisionSensor
from carla_interface.sensors.lane_invasion import LaneInvasionSensor
from carla_interface.sensors.gnss import GnssSensor
from carla_interface.sensors.imu import IMUSensor
from carla_interface.sensors.radar import RadarSensor
from carla_interface.sensors.camera import CameraManager, StereoCameraManager
from carla_interface.environment.world import World


class CarlaWorldNode(Node):
    """
    ROS 2 Node that initializes CARLA simulation and Pygame interface, and publishes stereo camera output.
    """
    def __init__(self):
        """
        Initializes CARLA client, Pygame display, sensors, and ROS 2 publishers.
        """
        super().__init__('carla_world_node')
        self.get_logger().info('Initializing CARLA + Pygame control loop...')

        self.args = self._parse_default_args()
        self.args.width, self.args.height = map(int, self.args.res.split('x'))

        # Connect to CARLA
        self.client = carla.Client(self.args.host, self.args.port)
        self.client.set_timeout(10.0)
        self.sim_world = self.client.get_world()
        self.traffic_manager = self.client.get_trafficmanager()

        self.original_carla_settings = self.sim_world.get_settings() # Store original settings

        if self.args.sync:
            self._enable_synchronous_mode()

        # Pygame UI
        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode(
            (self.args.width, self.args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF
        )

        # Initialize world
        self.hud = HUD(self.args.width, self.args.height)
        self.world = World(self.sim_world, self.hud, self.traffic_manager, self.args)
        self.controller = KeyboardControl(self.world, self.args.autopilot)
        self.clock = pygame.time.Clock()

        # ROS 2 Publishers
        self.left_pub = self.create_publisher(Image, "/carla/left/image_raw", 10)
        self.right_pub = self.create_publisher(Image, "/carla/right/image_raw", 10)
        self.left_info_pub = self.create_publisher(CameraInfo, "/carla/left/camera_info", 10)
        self.right_info_pub = self.create_publisher(CameraInfo, "/carla/right/camera_info", 10)

        self.timer = self.create_timer(0.05, self._tick)
        self._shutdown_requested = False # Flag to signal shutdown

        self.get_logger().info("CarlaWorldNode initialized.")

    def _parse_default_args(self):
        """
        Parses default arguments for connecting to the simulator and configuring the display.
        Allows specifying actor type via --filter.

        Returns:
            argparse.Namespace: A namespace containing simulation defaults.
        """
        parser = argparse.ArgumentParser(description="CARLA ROS 2 Pygame manual control client.")
        parser.add_argument(
            '--host',
            metavar='H',
            default='localhost',
            help='IP of the host server (default: localhost)')
        parser.add_argument(
            '-p', '--port',
            metavar='P',
            default=2000,
            type=int,
            help='TCP port to listen to (default: 2000)')
        parser.add_argument(
            '--res',
            metavar='WIDTHxHEIGHT',
            default='1000x500',
            help='Window resolution (default: 1000x500)')
        parser.add_argument(
            '--filter',
            metavar='PATTERN',
            # default='vehicle.*', # Default to vehicle, can be overridden to walker.pedestrian.*
            default='walker.pedestrian.*', # Default to vehicle, can be overridden to walker.pedestrian.*
            help='Actor filter (default: "vehicle.*")')
        parser.add_argument(
            '--generation',
            metavar='G',
            default='All',
            help='restrict to certain actor generation (values: "All", "1", "2", ...) (default: "All")')
        parser.add_argument(
            '--rolename',
            metavar='NAME',
            default='hero',
            help='Actor rolename (default: "hero")')
        parser.add_argument(
            '--gamma',
            default=1.0,
            type=float,
            help='Gamma correction of the camera (default: 1.0)')
        parser.add_argument(
            '--autopilot',
            action='store_true',
            help='enable autopilot')
        parser.add_argument(
            '--sync',
            action='store_true',
            default=True,
            help='Activate synchronous mode execution')

        # Allow unknown arguments, which will be ignored. This is useful for ros2 launch.
        args, unknown = parser.parse_known_args()
        return args

    def _enable_synchronous_mode(self):
        """
        Enables synchronous simulation mode in CARLA for deterministic behavior.
        """
        settings = self.sim_world.get_settings()
        if not settings.synchronous_mode:
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            self.sim_world.apply_settings(settings)
        self.traffic_manager.set_synchronous_mode(True)
        self.get_logger().info("Synchronous mode enabled.")


    def _disable_synchronous_mode(self):
        """
        Disables synchronous simulation mode and restores original settings.
        """
        if self.original_carla_settings:
            self.sim_world.apply_settings(self.original_carla_settings)
            self.traffic_manager.set_synchronous_mode(False)
            self.get_logger().info("Synchronous mode disabled and original CARLA settings restored.")


    def _tick(self):
        """
        Main control loop:
        - Synchronizes CARLA world tick
        - Handles keyboard events
        - Renders the world
        - Publishes stereo image and camera info
        """
        if self._shutdown_requested:
            return # Do not process further if shutdown is requested

        start_time = time.time()

        if self.args.sync:
            self.sim_world.tick()
        else:
            self.sim_world.wait_for_tick()
        after_tick = time.time()

        self.clock.tick_busy_loop(60)
        after_clock = time.time()

        if self.controller.parse_events(self.client, self.world, self.clock, self.args.sync):
            self.get_logger().info("Exit requested from keyboard (ESC). Signaling shutdown.")
            self._shutdown_requested = True # Set the flag
            return # Exit tick immediately

        after_events = time.time()

        self.world.tick(self.clock)
        after_world_tick = time.time()
        self.world.render(self.display)
        pygame.display.flip()
        after_render = time.time()

        # Publish stereo camera output
        self.world.stereocam.publish(
            self.left_pub, self.right_pub,
            self.left_info_pub, self.right_info_pub
        )

        # Log timings (optional, but good for debugging)
        self.get_logger().info(
            f"Tick time: {(after_render - start_time) * 1000:.1f} ms | "
            f"SimTick: {(after_tick - start_time) * 1000:.1f} ms | "
            f"Clock: {(after_clock - after_tick) * 1000:.1f} ms | "
            f"Events: {(after_events - after_clock) * 1000:.1f} ms | "
            f"WorldTick: {(after_world_tick - after_events) * 1000:.1f} ms | "
            f"Render: {(after_render - after_world_tick) * 1000:.1f} ms"
        )


    def destroy_node(self):
        """
        Custom destroy method to clean up CARLA and Pygame resources.
        """
        self.get_logger().info("Destroying CarlaWorldNode: Cleaning up resources...")
        self.timer.cancel() # Stop the ROS 2 timer

        if self.world:
            self.world.destroy() # Destroy CARLA actors and sensors
            self.get_logger().info("CARLA world actors and sensors destroyed.")

        if self.args.sync:
            self._disable_synchronous_mode() # Reset CARLA to async mode and original settings

        pygame.quit() # Quit Pygame
        self.get_logger().info("Pygame quit.")
        super().destroy_node() # Call the parent's destroy_node method
        self.get_logger().info("CarlaWorldNode fully destroyed.")


def main(args=None):
    """
    Entry point for launching the ROS 2 CarlaWorldNode.
    """
    rclpy.init(args=args)
    node = CarlaWorldNode()

    try:
        # Use a loop with spin_once to check the shutdown flag
        while rclpy.ok() and not node._shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1) # Process events for a short duration
            if node._shutdown_requested:
                node.get_logger().info("Shutdown requested, breaking spin loop.")
                break # Exit the loop if shutdown is requested

    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt detected. Signaling shutdown.")
        node._shutdown_requested = True # Ensure the flag is set on Ctrl+C
    except Exception as e:
        node.get_logger().error(f"An unexpected error occurred: {e}")
    finally:
        if rclpy.ok(): # Only destroy node if rclpy is still active
            node.get_logger().info("Cleaning up node resources...")
            node.destroy_node()
        else:
            node.get_logger().info("ROS 2 context already shut down. Skipping node destruction.")
        rclpy.shutdown() # Ensure ROS 2 context is fully shut down
        node.get_logger().info("Application shutdown complete.")


if __name__ == '__main__':
    main()