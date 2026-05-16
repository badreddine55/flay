"""Drone delivery simulation visualiser using pygame."""

import math
import random
from typing import Any

import pygame


class Drone:
    """Tracks one drone's animated position as it moves along a hub path."""

    def __init__(self) -> None:
        self.id: int = 0

        # Current screen position (pixels)
        self.x: float = 0.0
        self.y: float = 0.0

        # Endpoints of the current interpolated move
        self.last_x: float = 0.0
        self.last_y: float = 0.0
        self.next_x: float = 0.0
        self.next_y: float = 0.0

        # path = list of (hub_id, turn) steps
        # path_index = index of the *next* step we are heading toward
        self.path: list[tuple[str, int]] = []
        self.path_index: int = 1
        self.is_done: bool = False

        # Hub positions shared from Game (hub_id -> world coords)
        self.hubs_position: dict[str, tuple[float, float]] = {}

        # Small random offset so drones don't overlap when stacked
        self.wobble_x: float = float(random.randint(-10, 10))
        self.wobble_y: float = float(random.randint(-10, 10))

        # Map transform injected by Game after compute_scale()
        self.map_scale: float = 1.0
        self.offset_x: float = 0.0
        self.offset_y: float = 0.0
        self.min_x: float = 0.0
        self.min_y: float = 0.0

    def world_to_screen(
        self, wx: float, wy: float
    ) -> tuple[float, float]:
        """Convert world coordinates to screen pixels."""
        sx = (wx - self.min_x) * self.map_scale + self.offset_x
        sy = (wy - self.min_y) * self.map_scale + self.offset_y
        return sx, sy

    def get_screen_pos_at(self, index: int) -> tuple[float, float]:
        """Return the screen position for path step at index.

        If the hub_id is not a real hub it is a transit waypoint; in that
        case we return the midpoint between the previous and next real hubs.
        """
        hub_id = self.path[index][0]

        if hub_id in self.hubs_position:
            return self.world_to_screen(*self.hubs_position[hub_id])

        # Transit waypoint: average the neighbours
        prev_hub_id = self.path[index - 1][0]
        next_hub_id = self.path[index + 1][0]
        px, py = self.world_to_screen(*self.hubs_position[prev_hub_id])
        nx, ny = self.world_to_screen(*self.hubs_position[next_hub_id])
        return (px + nx) / 2.0, (py + ny) / 2.0

    def _is_waiting_between_turns(self, current_turn: int) -> bool:
        """Return True when the drone has
        arrived but next move is not due yet."""
        if self.path_index >= len(self.path):
            return False
        due_turn = self.path[self.path_index][1]
        prev_turn = self.path[self.path_index - 1][1]
        return prev_turn < current_turn < due_turn

    def update(self, current_turn: int, frame_count: int) -> None:
        """Advance the drone animation by one frame."""
        if self.is_done:
            return

        frames_per_turn: int = 60
        t = frame_count / frames_per_turn
        t = max(0.0, min(1.0, t))

        # Smoothstep easing: slow at start and end, fast in the middle
        smooth_t = t * t * (3.0 - 2.0 * t)

        if frame_count == 59 and self.path_index >= len(self.path):
            self.is_done = True
            return

        if (
            self.path_index < len(self.path)
            and current_turn == self.path[self.path_index][1]
        ):
            self.last_x, self.last_y = self.x, self.y
            self.next_x, self.next_y = self.get_screen_pos_at(
                self.path_index
            )
            self.path_index += 1

        if self._is_waiting_between_turns(current_turn):
            return

        dx = (self.next_x - self.last_x) * smooth_t
        dy = (self.next_y - self.last_y) * smooth_t
        self.x = self.last_x + dx + self.wobble_x
        self.y = self.last_y + dy + self.wobble_y


class Game:
    """Main simulation: loads data, draws the map and drones, runs the loop."""

    def __init__(self) -> None:
        pygame.init()
        info = pygame.display.Info()
        self.WIDTH: int = info.current_w
        self.HEIGHT: int = info.current_h

        self.UI_SCALE: float = min(self.WIDTH / 1800.0, self.HEIGHT / 900.0)

        self.screen = pygame.display.set_mode(
            (self.WIDTH, self.HEIGHT), pygame.RESIZABLE
        )
        pygame.display.set_caption("Drone Simulation")

        self.clock: pygame.time.Clock = pygame.time.Clock()
        self.FPS: int = 60
        self.running: bool = True

        self.data: Any = None
        self.drones: list[Drone] = []
        self.hubs_position: dict[str, tuple[float, float]] = {}

        # Element sizes in pixels
        self.HUB_RADIUS: int = int(30 * self.UI_SCALE)
        self.DRONE_SIZE: int = int(24 * self.UI_SCALE)
        self.PADDING: int = max(self.HUB_RADIUS, self.DRONE_SIZE) + 20

        # Map transform computed in _compute_scale()
        self.map_scale: float = 1.0
        self.offset_x: float = 0.0
        self.offset_y: float = 0.0
        self.min_x: float = 0.0
        self.min_y: float = 0.0
        self.max_x: float = 0.0
        self.max_y: float = 0.0

        # Simulation time
        self.current_turn: int = 0
        self.frame_count: int = 0  # counts 0-59 within each turn

        self.bg = pygame.image.load(
            "images/congruent_pentagon.png"
        ).convert_alpha()
        self.bg = pygame.transform.scale(self.bg, (self.WIDTH, self.HEIGHT))

        self.drone_image = pygame.image.load(
            "images/drone.png").convert_alpha()
        self.drone_image = pygame.transform.scale(
            self.drone_image,
            (int(40 * self.UI_SCALE), int(40 * self.UI_SCALE)),
        )

        self.hub_font = pygame.font.SysFont(
            "Arial", int(20 * self.UI_SCALE), bold=True
        )
        self.turn_font = pygame.font.SysFont(
            "Arial", int(30 * self.UI_SCALE), bold=True
        )
        self.cap_font = pygame.font.SysFont(
            "Arial", int(12 * self.UI_SCALE), bold=True
        )

    def load_map(self, data: Any) -> None:
        """Load hub/zone data, compute world bounds and map scale."""
        self.data = data
        for hub in data.zones:
            self.hubs_position[hub.name] = hub.coordinates
        self._compute_bounds()
        self._compute_scale()

    def load_drones(self, drones: Any) -> None:
        """Create Drone objects from simulation data
        and place them at start."""
        for drone_data in drones:
            drone = Drone()
            drone.id = drone_data.id
            drone.path = drone_data.path
            drone.hubs_position = self.hubs_position
            drone.map_scale = self.map_scale
            drone.offset_x = self.offset_x
            drone.offset_y = self.offset_y
            drone.min_x = self.min_x
            drone.min_y = self.min_y

            start_hub = drone_data.path[0][0]
            sx, sy = self.world_to_screen(*self.hubs_position[start_hub])
            drone.x = drone.last_x = drone.next_x = float(sx)
            drone.y = drone.last_y = drone.next_y = float(sy)

            self.drones.append(drone)

    def _compute_bounds(self) -> None:
        """Find the min/max world coordinates across all hubs."""
        coords: list[tuple[float, float]] = [
            hub.coordinates for hub in self.data.zones
        ]
        self.min_x = min(c[0] for c in coords)
        self.min_y = min(c[1] for c in coords)
        self.max_x = max(c[0] for c in coords)
        self.max_y = max(c[1] for c in coords)

    def _compute_scale(self) -> None:
        """Compute map_scale and offsets so all hubs fit inside screen."""
        usable_w = self.WIDTH - 2 * self.PADDING
        usable_h = self.HEIGHT - 3 * self.PADDING
        range_x = self.max_x - self.min_x
        range_y = self.max_y - self.min_y

        if range_x == 0 and range_y == 0:
            self.map_scale = float(min(usable_w, usable_h))
        elif range_x == 0:
            self.map_scale = usable_h / range_y
        elif range_y == 0:
            self.map_scale = usable_w / range_x
        else:
            self.map_scale = min(
                usable_w / range_x, usable_h / range_y
            )

        self.offset_x = (
            self.PADDING
            + (usable_w - range_x * self.map_scale) / 2.0
        )
        self.offset_y = (
            self.PADDING
            + (usable_h - range_y * self.map_scale) / 2.0
        )

    def world_to_screen(self, wx: float, wy: float) -> tuple[int, int]:
        """Convert world coordinates to integer screen pixels."""
        sx = int((wx - self.min_x) * self.map_scale + self.offset_x)
        sy = int((wy - self.min_y) * self.map_scale + self.offset_y)
        return sx, sy

    def advance_time(self) -> None:
        """Increment the frame counter; roll over to a new turn at 60 fps."""
        self.frame_count += 1
        if self.frame_count >= 60:
            self.frame_count = 0
            self.current_turn += 1

    def _draw_connections(self) -> None:
        """Draw lines between connected hubs with capacity labels."""
        for hub in self.data.zones:
            sx, sy = self.world_to_screen(*hub.coordinates)
            for entry in self.data.get_neighbors(hub.name):
                ex, ey = self.world_to_screen(*entry.neighbor.coordinates)

                if math.hypot(ex - sx, ey - sy) == 0:
                    continue

                pygame.draw.line(
                    self.screen,
                    (60, 60, 60),
                    (sx, sy),
                    (ex, ey),
                    max(1, int(7 * self.UI_SCALE)),
                )

                mid_x = (sx + ex) // 2
                mid_y = (sy + ey) // 2
                capacity = str(entry.connection.max_link_capacity)
                cap_label = self.cap_font.render(
                    capacity, True, (255, 255, 0)
                )
                cap_rect = cap_label.get_rect(center=(mid_x, mid_y))
                bg_rect = cap_rect.inflate(3, 3)
                pygame.draw.rect(
                    self.screen, (30, 30, 30), bg_rect, border_radius=3
                )
                self.screen.blit(cap_label, cap_rect)

    def _draw_hubs(self) -> None:
        """Draw hub circles with their max-drone-count label."""
        for hub in self.data.zones:
            sx, sy = self.world_to_screen(*hub.coordinates)
            pygame.draw.circle(
                self.screen,
                self.color_name_to_rgb(hub.color),
                (sx, sy),
                self.HUB_RADIUS,
            )
            pygame.draw.circle(
                self.screen,
                (255, 255, 255),
                (sx, sy),
                self.HUB_RADIUS - int(6 * self.UI_SCALE),
                max(1, int(4 * self.UI_SCALE)),
            )
            label = self.hub_font.render(
                str(hub.max_drones), True, (255, 255, 255)
            )
            self.screen.blit(label, label.get_rect(center=(sx, sy)))

    def _draw_turn_counter(self) -> None:
        """Render the current turn number in the top-right corner."""
        label = self.turn_font.render(
            str(self.current_turn), True, pygame.Color("black")
        )
        self.screen.blit(
            label, label.get_rect(topright=(self.WIDTH - 20, 20))
        )

    def _draw_drones(self) -> None:
        """Update and draw every drone sprite."""
        half_w = self.drone_image.get_width() // 2
        half_h = self.drone_image.get_height() // 2
        for drone in self.drones:
            drone.update(self.current_turn, self.frame_count)
            self.screen.blit(
                self.drone_image,
                (int(drone.x) - half_w, int(drone.y) - half_h),
            )

    def run(self) -> None:
        """Start the simulation loop. Blocks until the window is closed."""
        while self.running:
            self.clock.tick(self.FPS)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            self.screen.blit(self.bg, (0, 0))
            self._draw_connections()
            self._draw_hubs()
            self._draw_turn_counter()
            self._draw_drones()
            pygame.display.flip()

            if not all(drone.is_done for drone in self.drones):
                self.advance_time()

        pygame.quit()

    @staticmethod
    def color_name_to_rgb(color_name: str) -> tuple[int, int, int]:
        """Convert a pygame color name to an (R, G, B) tuple.

        Falls back to black for the special value 'rainbow'.
        """
        resolved = "black" if color_name == "rainbow" else color_name
        c = pygame.Color(resolved)
        return c.r, c.g, c.b
