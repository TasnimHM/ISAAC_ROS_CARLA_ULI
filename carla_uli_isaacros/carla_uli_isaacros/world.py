import sys
import random
import carla

from .util import find_weather_presets, get_actor_display_name, get_actor_blueprints
from .sensors.collision import CollisionSensor
from .sensors.lane_invasion import LaneInvasionSensor
from .sensors.gnss import GnssSensor
from .sensors.imu import IMUSensor
from .sensors.radar import RadarSensor
from .sensors.camera import CameraManager, StereoCameraManager


class World:
    def __init__(self, carla_world, hud, traffic_manager, args):
        # Core attributes
        self.world = carla_world
        self.hud = hud
        self.traffic_manager = traffic_manager
        self.sync = args.sync
        self.actor_role_name = args.rolename
        self._actor_filter = args.filter
        self._actor_generation = args.generation
        self._gamma = args.gamma

        # Simulation settings
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

        # Weather
        self._weather_presets = find_weather_presets()
        self._weather_index = 0

        # Core actor and sensors
        self.player = None
        self.camera_manager = None
        self.radar_sensor = None

        try:
            self.map = self.world.get_map()
        except RuntimeError as e:
            print(f"RuntimeError: {e}")
            sys.exit(1)

        self.restart()
        self.world.on_tick(hud.on_world_tick)

    def restart(self):
        cam_index = self.camera_manager.index if self.camera_manager else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager else 0

        # Spawn the player
        self._spawn_player()

        # Modify vehicle physics
        self.modify_vehicle_physics(self.player)

        # Initialize sensors
        self.stereocam = StereoCameraManager(self.player)
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma)
        self.camera_manager.set_sensor(cam_index, notify=False)
        self.camera_manager.transform_index = cam_pos_index

        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)
        self.radar_sensor = None  # Optional: assign if needed

        # Notify HUD and traffic manager
        self.hud.notification(get_actor_display_name(self.player))
        self.traffic_manager.update_vehicle_lights(self.player, True)

        # Tick once for sync
        if self.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def _spawn_player(self):
        blueprint_list = get_actor_blueprints(self.world, self._actor_filter, self._actor_generation)
        if not blueprint_list:
            raise ValueError("No valid vehicle blueprint found.")
        blueprint = random.choice(blueprint_list)
        blueprint.set_attribute('role_name', self.actor_role_name)

        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        if self.player:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        else:
            spawn_points = self.map.get_spawn_points()
            if not spawn_points:
                sys.exit("No spawn points available.")
            spawn_point = random.choice(spawn_points)
            spawn_point.location.z += 2.0
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)

    def modify_vehicle_physics(self, actor):
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception:
            pass

    def destroy(self):
        if self.radar_sensor:
            self.radar_sensor.sensor.destroy()

        for sensor in [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.imu_sensor.sensor
        ]:
            if sensor:
                sensor.stop()
                sensor.destroy()

        if self.player:
            self.player.destroy()

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def next_weather(self, reverse=False):
        self._weather_index = (self._weather_index - 1 if reverse else self._weather_index + 1) % len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification(f'Weather: {preset[1]}')
        self.player.get_world().set_weather(preset[0])
