import sys
import time
import random
import argparse

import pygame
import carla
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo

# Local project modules
from carla_interface.utils.hud import HUD
from carla_interface.utils.keyboard_control import KeyboardControl
from carla_interface.utils.util import find_weather_presets, get_actor_display_name, get_actor_blueprints
from carla_interface.sensors.collision import CollisionSensor
from carla_interface.sensors.lane_invasion import LaneInvasionSensor
from carla_interface.sensors.gnss import GnssSensor
from carla_interface.sensors.imu import IMUSensor
from carla_interface.sensors.radar import RadarSensor
from carla_interface.sensors.camera import CameraManager, StereoCameraManager # Ensure StereoCameraManager has a destroy method

class World:
    """
    Manages the CARLA world, including player vehicle, sensors, weather, and rendering.
    """
    def __init__(self, carla_world, hud, traffic_manager, args):
        """
        Initializes the simulation world, weather, player, and HUD.

        Args:
            carla_world: CARLA world instance.
            hud: Heads-up display object.
            traffic_manager: CARLA traffic manager.
            args: Simulation settings from CLI or defaults.
        """
        self.world = carla_world
        self.hud = hud
        self.traffic_manager = traffic_manager
        self.sync = args.sync
        self.actor_role_name = args.rolename
        self._actor_filter = args.filter
        self._actor_generation = args.generation
        self._gamma = args.gamma

        # Flags
        self.constant_velocity_enabled = False
        self.recording_enabled = False
        self.recording_start = 0
        self.show_vehicle_telemetry = False
        self.doors_are_open = False

        # Map layers
        self.current_map_layer = 0
        self.map_layer_names = [
            carla.MapLayer.NONE,
            carla.MapLayer.Buildings,
            carla.MapLayer.Decals,
            carla.MapLayer.Foliage,
            carla.MapLayer.Ground,
            carla.MapLayer.ParkedVehicles,
            carla.MapLayer.Particles,
            carla.MapLayer.Props,
            carla.MapLayer.StreetLights,
            carla.MapLayer.Walls,
            carla.MapLayer.All
        ]

        # Weather presets
        self._weather_presets = find_weather_presets()
        self._weather_index = 0

        # Actor and sensors - Initialize to None here
        self.player = None
        self.camera_manager = None
        self.stereocam = None # Initialize stereocam here
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.imu_sensor = None
        self.radar_sensor = None

        try:
            self.map = self.world.get_map()
        except RuntimeError as e:
            print(f"RuntimeError: {e}")
            sys.exit(1)

        self.restart()
        self.world.on_tick(hud.on_world_tick)

    def restart(self):
        """
        Respawns the player vehicle and resets all sensors and HUD state.
        This method will also destroy previous actors if they exist before spawning new ones.
        """
        # Store current camera state before destroying
        cam_index = self.camera_manager.index if self.camera_manager else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager else 0

        # --- IMPORTANT: Destroy existing actors before spawning new ones during restart ---
        self.destroy() # Call destroy to clean up previous run's actors

        self._spawn_player()
        self.modify_vehicle_physics(self.player)

        # Create new sensor instances
        # Ensure StereoCameraManager also tracks its CARLA actors internally
        self.stereocam = StereoCameraManager(self.player) # Pass self.world if Stereocam needs to spawn
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma)
        self.camera_manager.set_sensor(cam_index, notify=False)
        self.camera_manager.transform_index = cam_pos_index

        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)
        self.radar_sensor = None  # Optional

        self.hud.notification(get_actor_display_name(self.player))
        self.traffic_manager.update_vehicle_lights(self.player, True)

        if self.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def _spawn_player(self):
        """
        Spawns the player vehicle using a filtered blueprint and sets its role and color.
        """
        blueprints = get_actor_blueprints(self.world, self._actor_filter, self._actor_generation)
        if not blueprints:
            raise ValueError("No valid vehicle blueprint found.")
        blueprint = random.choice(blueprints)
        blueprint.set_attribute('role_name', self.actor_role_name)

        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        spawn_point = None
        if self.player: # This branch is mostly for respawn within the same run, but 'restart' handles initial destroy
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            # self.destroy() # This destroy call is now redundant because restart() calls destroy() at the beginning
        else:
            spawn_points = self.map.get_spawn_points()
            if not spawn_points:
                sys.exit("No spawn points available.")
            spawn_point = random.choice(spawn_points)
            spawn_point.location.z += 2.0

        self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        if self.player is None:
            raise RuntimeError("Failed to spawn player actor.")


    def modify_vehicle_physics(self, actor):
        """
        Modifies vehicle physics (e.g., enables sweep wheel collision).

        Args:
            actor: The CARLA vehicle actor to modify.
        """
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception as e:
            # print(f"Warning: Could not modify vehicle physics: {e}")
            pass # Suppress warning if physics control isn't supported for actor type

    def destroy(self):
        """
        Destroys all active sensors and the player actor.
        This method is crucial for proper CARLA cleanup.
        """
        # List of all sensor instances that might exist
        sensors_to_destroy = [
            self.camera_manager,
            self.stereocam, # Add StereoCameraManager here
            self.collision_sensor,
            self.lane_invasion_sensor,
            self.gnss_sensor,
            self.imu_sensor,
            self.radar_sensor
        ]

        # Iterate through sensor managers and call their destroy methods
        # Each sensor manager (e.g., CameraManager, StereoCameraManager, CollisionSensor)
        # must have a .destroy() method that takes care of its internal carla.Sensor actor.
        for sensor_manager in sensors_to_destroy:
            if sensor_manager is not None:
                # Check if the manager has a 'sensor' attribute (for simple sensors)
                # or a 'destroy' method (for more complex managers like CameraManager)
                if hasattr(sensor_manager, 'sensor') and sensor_manager.sensor and sensor_manager.sensor.is_alive:
                    sensor_manager.sensor.stop()
                    sensor_manager.sensor.destroy()
                    sensor_manager.sensor = None # Clear reference
                elif hasattr(sensor_manager, 'destroy') and callable(getattr(sensor_manager, 'destroy')):
                    sensor_manager.destroy() # Call the manager's own destroy method
                # After destroying the CARLA actor, clear the reference to the manager itself
                if sensor_manager == self.camera_manager: self.camera_manager = None
                if sensor_manager == self.stereocam: self.stereocam = None
                if sensor_manager == self.collision_sensor: self.collision_sensor = None
                if sensor_manager == self.lane_invasion_sensor: self.lane_invasion_sensor = None
                if sensor_manager == self.gnss_sensor: self.gnss_sensor = None
                if sensor_manager == self.imu_sensor: self.imu_sensor = None
                if sensor_manager == self.radar_sensor: self.radar_sensor = None

        # Finally, destroy the player actor
        if self.player and self.player.is_alive:
            self.player.destroy()
            self.player = None # Clear reference
            # print("Player destroyed.") # For debugging
        # else:
            # print("Player not found or already destroyed.") # For debugging

    def tick(self, clock):
        """
        Advances HUD and world tick.

        Args:
            clock: Pygame clock to maintain real-time pacing.
        """
        self.hud.tick(self, clock)

    def render(self, display):
        """
        Renders the camera and HUD to the display.

        Args:
            display: Pygame display surface.
        """
        if self.camera_manager:
            self.camera_manager.render(display)
        if self.hud:
            self.hud.render(display)

    def next_weather(self, reverse=False):
        """
        Cycles through predefined weather presets.

        Args:
            reverse (bool): If True, cycles backward.
        """
        self._weather_index = (self._weather_index - 1 if reverse else self._weather_index + 1) % len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification(f'Weather: {preset[1]}')
        self.player.get_world().set_weather(preset[0])