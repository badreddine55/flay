from file_parser import Connection, Zone, ZoneType


class AdjacencyEntry:
    """One neighbor entry in the adjacency list."""

    def __init__(
        self, neighbor_zone: Zone, cost: float, connection: Connection
    ) -> None:
        self.neighbor: Zone = neighbor_zone
        self.cost: float = cost
        self.connection: Connection = connection


class AdjacencyList:
    """Represents the graph as a dictionary of neighbor lists."""

    def __init__(self) -> None:
        self._data: dict[str, list[AdjacencyEntry]] = {}
        self.zones: list[Zone] = []

    def add_zone(self, zone: Zone) -> None:
        """Register a zone using its name as key."""
        if zone.name not in self._data:
            self._data[zone.name] = []
            self.zones.append(zone)

    def add_connection(
        self, zone_a: Zone, zone_b: Zone, connection: Connection
    ) -> None:
        """Add a bidirectional connection between two Zone objects."""
        cost_a_to_b = self._get_cost(zone_b)
        cost_b_to_a = self._get_cost(zone_a)

        self._data[zone_a.name].append(
            AdjacencyEntry(zone_b, cost_a_to_b, connection)
        )
        self._data[zone_b.name].append(
            AdjacencyEntry(zone_a, cost_b_to_a, connection)
        )

    def get_neighbors(self, zone_name: str) -> list[AdjacencyEntry]:
        """Return all neighbors of a zone by name."""
        return self._data.get(zone_name, [])

    def get_neighbor_entries(self, zone: Zone | str) -> list[AdjacencyEntry]:
        """Return raw AdjacencyEntry objects (for
        algorithms that need cost/connection)."""
        name = zone.name if isinstance(zone, Zone) else zone
        return self._data.get(name, [])

    def _get_cost(self, zone: Zone) -> int:
        """Calculate movement cost based on destination zone type."""
        if zone.zone_type == ZoneType.RESTRICTED:
            return 2
        if zone.zone_type == ZoneType.BLOCKED:
            return 999
        return 1
