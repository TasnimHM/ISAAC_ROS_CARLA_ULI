import pygame
import carla
import math

class KeyboardControl:
    def __init__(self, world, start_in_autopilot=False):
        self._autopilot_enabled = start_in_autopilot
        self.world = world
        self.clock = None  # Will be set at runtime

        self.actor = world.player
        self.is_vehicle = isinstance(self.actor, carla.Vehicle)
        self.control = carla.VehicleControl() if self.is_vehicle else carla.WalkerControl()
        self._steer_cache = 0.0
        self.yaw = self.actor.get_transform().rotation.yaw if not self.is_vehicle else None

        if self.is_vehicle:
            try:
                self.actor.set_autopilot(self._autopilot_enabled)
            except:
                print("Failed to set autopilot for vehicle.")
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self, client, world, clock, sync_mode):
        self.clock = clock

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if event.key == pygame.K_ESCAPE:
                    return True
                elif event.key == pygame.K_BACKSPACE:
                    world.restart()
                elif event.key == pygame.K_h:
                    world.hud.help.toggle()
                elif event.key == pygame.K_TAB and hasattr(world, 'camera_manager'):
                    world.camera_manager.toggle_camera()
                elif event.key == pygame.K_c:
                    world.next_weather()
                elif event.key == pygame.K_r and hasattr(world, 'camera_manager'):
                    world.camera_manager.toggle_recording()
                elif event.key == pygame.K_p and self.is_vehicle:
                    self._autopilot_enabled = not self._autopilot_enabled
                    self.actor.set_autopilot(self._autopilot_enabled)
                    world.hud.notification(f"Autopilot {'On' if self._autopilot_enabled else 'Off'}")

        keys = pygame.key.get_pressed()
        if self.is_vehicle:
            self._parse_vehicle_keys(keys, clock.get_time())
        else:
            self._parse_walker_keys(keys)

        try:
            self.actor.apply_control(self.control)
        except Exception as e:
            world.hud.notification(f"Failed to apply control: {e}")

        return False

    def _parse_vehicle_keys(self, keys, milliseconds):
        if keys[pygame.K_UP] or keys[pygame.K_w]:
            self.control.throttle = min(self.control.throttle + 0.1, 1.00)
        else:
            self.control.throttle = 0.0

        if keys[pygame.K_DOWN] or keys[pygame.K_s]:
            self.control.brake = min(self.control.brake + 0.2, 1)
        else:
            self.control.brake = 0

        steer_increment = 5e-4 * milliseconds
        if keys[pygame.K_LEFT] or keys[pygame.K_a]:
            self._steer_cache = max(-0.7, self._steer_cache - steer_increment)
        elif keys[pygame.K_RIGHT] or keys[pygame.K_d]:
            self._steer_cache = min(0.7, self._steer_cache + steer_increment)
        else:
            self._steer_cache = 0.0

        self.control.steer = round(self._steer_cache, 1)
        self.control.hand_brake = keys[pygame.K_SPACE]

    def _parse_walker_keys(self, keys):
        speed = 1.5  # m/s

        if keys[pygame.K_UP] or keys[pygame.K_w]:
            self.control.speed = speed
        elif keys[pygame.K_DOWN] or keys[pygame.K_s]:
            self.control.speed = -speed
        else:
            self.control.speed = 0.0

        # Handle turning (update yaw)
        yaw_rate = 2.0  # degrees per frame
        if keys[pygame.K_LEFT] or keys[pygame.K_a]:
            self.yaw -= yaw_rate
        elif keys[pygame.K_RIGHT] or keys[pygame.K_d]:
            self.yaw += yaw_rate

        rad = math.radians(self.yaw)
        self.control.direction = carla.Vector3D(x=math.cos(rad), y=math.sin(rad), z=0.0)
