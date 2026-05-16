from file_parser import ZoneType
from collections import defaultdict
from typing import Any
import heapq


class Drone:
    def __init__(self, drone_id: str, position: str) -> None:
        self.id = drone_id
        self.position = position
        self.path: list[Any] = []


class Dijkstra:
    def __init__(self, graph: Any, zones: Any) -> None:
        self.graph = graph
        self.zones = zones
        self.hub_cache: dict[Any, int] = {}
        self.link_cache: dict[Any, int] = {}

    def _reconstruct_path(
        self,
        prev: dict[Any, Any],
        turns: dict[Any, Any],
        start: str,
        end: str,
    ) -> list[Any]:
        path = []
        node = end

        if prev[node] is None and node != start:
            return []

        while node is not None:
            path.append((node, turns[node], "zone"))

            if prev[node] is not None:
                destination_zone = self.zones[node]
                if destination_zone.zone_type == ZoneType.RESTRICTED:
                    connection_name = f"{prev[node]}-{node}"
                    transit_turn = turns[node] - 1
                    path.append((connection_name, transit_turn, "transit"))

            node = prev[node]

        path.reverse()
        return path

    def find_path(
        self, drone: Drone, start: str, end: str, current_turn: int
    ) -> list[Any]:
        heap: list[Any] = []
        dist = {zone: float('inf') for zone in self.graph._data}
        dist[start] = 0
        prev = {zone: None for zone in self.graph._data}
        turns = {zone: 0 for zone in self.graph._data}
        turns[start] = current_turn
        visited = set()

        heapq.heappush(heap, (0, current_turn, start))

        while heap:
            cost, turn, node = heapq.heappop(heap)

            if (node, turn) in visited:
                continue

            visited.add((node, turn))

            if node == end:
                break

            neighbors = self.graph.get_neighbors(node)

            for entry in neighbors:
                neighbor = entry.neighbor

                if neighbor.zone_type == ZoneType.BLOCKED:
                    continue

                # priority zones cost half → dijkstra naturally prefers them
                is_priority = neighbor.zone_type == ZoneType.PRIORITY
                cost_modifier = 0.5 if is_priority else 1

                arrival_turn = turn + entry.cost
                zone_key = (neighbor.name, arrival_turn)
                link_key = (
                    min(node, neighbor.name),
                    max(node, neighbor.name),
                    turn,
                )

                # how many drones already booked this zone and link
                # at this turn
                reserved_zone = self.hub_cache.get(zone_key, 0)
                reserved_link = self.link_cache.get(link_key, 0)

                if reserved_link >= entry.connection.max_link_capacity:
                    continue

                if reserved_zone >= neighbor.max_drones:
                    continue

                # found a better path to this neighbor → update and push
                new_cost = cost + (entry.cost * cost_modifier)

                if new_cost < dist[neighbor.name]:
                    dist[neighbor.name] = new_cost
                    prev[neighbor.name] = node
                    turns[neighbor.name] = arrival_turn

                    heapq.heappush(
                        heap,
                        (new_cost, arrival_turn, neighbor.name),
                    )

            # ── WAIT OPTION ────────────────────────────────────────────
            # after trying all neighbors, always offer "stay here one
            # more turn"
            # this is the fallback if every neighbor was full or blocked
            wait_turn = turn + 1
            current_zone = self.zones[node]
            reserved_wait = self.hub_cache.get((node, wait_turn), 0)

            if reserved_wait < current_zone.max_drones:
                heapq.heappush(heap, (cost + 1, wait_turn, node))

        path = self._reconstruct_path(prev, turns, start, end)

        for zone, turn, state in path:
            if state == "zone":
                key = (zone, turn)
                self.hub_cache[key] = self.hub_cache.get(key, 0) + 1

        for i in range(len(path) - 1):
            zone_a, _, state_a = path[i]
            _, turn_b, state_b = path[i + 1]

            # transit markers are not real zones → skip them
            if state_a == "transit" or state_b == "transit":
                continue

            # departure turn = when the drone LEFT zone_a
            # (one turn before arriving at zone_b)
            departure_turn = turn_b - 1
            zone_b = path[i + 1][0]

            link_key = (
                min(zone_a, zone_b),
                max(zone_a, zone_b),
                departure_turn,
            )
            self.link_cache[link_key] = (
                self.link_cache.get(link_key, 0) + 1
            )

        return path


class Simulation:
    def __init__(self, graph: Any, drones: list[Drone]) -> None:
        self.graph = graph
        self.drones = drones

    def run(self) -> None:
        movements: dict[Any, list[Any]] = defaultdict(list)

        for drone in self.drones:
            for (zone, turn, state) in drone.path:
                if turn == 0:
                    continue
                movements[turn].append((drone.id, zone))

        for turn in sorted(movements.keys()):
            line = ""
            for drone_id, zone in movements[turn]:
                line += f"{drone_id}-{zone} "
            print(line.strip())
        print(self.drones[-1].path[-1][1])
