import pygame
import carla

class KeyboardControl:
    def __init__(self, world, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        self._control = carla.VehicleControl()
        self._steer_cache = 0.0
        try:
            world.player.set_autopilot(self._autopilot_enabled)
        except:
            print("Failed to set autopilot mode. Ensure the player is a vehicle.")
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self, client, world, clock, sync_mode):
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
                elif event.key == pygame.K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == pygame.K_c:
                    world.next_weather()
                elif event.key == pygame.K_g:
                    world.toggle_radar()
                elif event.key == pygame.K_r:
                    world.camera_manager.toggle_recording()
                elif event.key == pygame.K_p:
                    self._autopilot_enabled = not self._autopilot_enabled
                    world.player.set_autopilot(self._autopilot_enabled)
                    world.hud.notification(f"Autopilot {'On' if self._autopilot_enabled else 'Off'}")

        keys = pygame.key.get_pressed()
        if isinstance(self._control, carla.VehicleControl):
            self._parse_vehicle_keys(keys, clock.get_time())
            try:
                world.player.apply_control(self._control)
            except:
                world.hud.notification("Failed to apply control. Ensure the player is a vehicle.")
        return False

    def _parse_vehicle_keys(self, keys, milliseconds):
        if keys[pygame.K_UP] or keys[pygame.K_w]:
            self._control.throttle = min(self._control.throttle + 0.1, 1.00)
        else:
            self._control.throttle = 0.0

        if keys[pygame.K_DOWN] or keys[pygame.K_s]:
            self._control.brake = min(self._control.brake + 0.2, 1)
        else:
            self._control.brake = 0

        steer_increment = 5e-4 * milliseconds
        if keys[pygame.K_LEFT] or keys[pygame.K_a]:
            self._steer_cache = max(-0.7, self._steer_cache - steer_increment)
        elif keys[pygame.K_RIGHT] or keys[pygame.K_d]:
            self._steer_cache = min(0.7, self._steer_cache + steer_increment)
        else:
            self._steer_cache = 0.0

        self._control.steer = round(self._steer_cache, 1)
        self._control.hand_brake = keys[pygame.K_SPACE]
