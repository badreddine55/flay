import sys
from enum import Enum
from typing import Generator, Optional


class ParseError(Exception):
    """Raised when the map file contains a syntax or semantic error.

    Attributes:
        line_number: The 1-based line number where the error occurred.
        cause:       A human-readable description of the problem.
    """

    def __init__(self, line_number: int, cause: str) -> None:
        super().__init__(f"Line {line_number}: {cause}")
        self.line_number = line_number
        self.cause = cause


class ZoneType(Enum):
    """Movement cost and accessibility of a zone.

    NORMAL:     Standard zone — costs 1 turn to enter.
    BLOCKED:    Impassable — no drone may enter or pass through.
    RESTRICTED: Sensitive zone — costs 2 turns to enter.
    PRIORITY:   Preferred zone — costs 1 turn but favoured by pathfinding.
    """

    NORMAL = "normal"
    BLOCKED = "blocked"
    RESTRICTED = "restricted"
    PRIORITY = "priority"


class Zone:
    """A single node (hub) in the drone network.

    Attributes:
        name:        Unique identifier — no dashes or spaces allowed.
        coordinates: (x, y) integer position on the map.
        zone_type:   Movement cost / accessibility type.
        color:       Optional single-word display colour.
        max_drones:  Maximum drones that may occupy this zone at once.
        role:        One of ``"hub"``, ``"start"``, or ``"end"``.
    """

    def __init__(
        self,
        name: str,
        coordinates: tuple[int, int],
        zone_type: ZoneType = ZoneType.NORMAL,
        color: Optional[str] = None,
        max_drones: int = 1,
        role: str = "hub",
    ) -> None:
        self.name = name
        self.coordinates = coordinates
        self.zone_type = zone_type
        self.color = color
        self.max_drones = max_drones
        self.role = role


class Connection:
    """A bidirectional edge between two zones.

    Attributes:
        zone1:             Name of the first zone.
        zone2:             Name of the second zone.
        max_link_capacity: Maximum drones that may traverse this edge at once.
    """

    def __init__(
        self,
        zone1: str,
        zone2: str,
        max_link_capacity: int = 1,
    ) -> None:
        self.zone1 = zone1
        self.zone2 = zone2
        self.max_link_capacity = max_link_capacity


class Graph:
    """Complete in-memory representation of a parsed drone network.

    Attributes:
        nb_drones:   Total drones that must travel start → end.
        zones:       Mapping of zone name → Zone (includes start and end).
        connections: All edges in the network.
        start:       The unique departure zone.
        end:         The unique destination zone.
    """

    def __init__(
        self,
        nb_drones: int,
        zones: dict[str, Zone],
        connections: list[Connection],
        start: Zone,
        end: Zone,
    ) -> None:
        self.nb_drones = nb_drones
        self.zones = zones
        self.connections = connections
        self.start = start
        self.end = end


class Parser:
    """Reads and validates a .map file, producing a Graph object.

    Usage::

        graph = Parser("maps/example.map").parse()

    Attributes:
        file_path: Path to the .map file supplied at construction time.
    """

    def __init__(self, file_path: str) -> None:
        self.file_path = file_path

    def parse(self) -> Graph:
        """Parse the file and return a validated Graph.

        Raises:
            ParseError:        On any syntax or semantic problem in the file.
            FileNotFoundError: If the file path does not exist.
        """
        zones: dict[str, Zone] = {}
        connections: list[Connection] = []
        seen_connections: set[tuple[str, str]] = set()
        start_zone: Optional[Zone] = None
        end_zone: Optional[Zone] = None
        nb_drones: int = 0
        nb_drones_parsed = False

        for line_number, raw_line in self._read_lines():
            line = raw_line.strip()

            # Skip blanks and comments.
            if not line or line.startswith("#"):
                continue

            # The very first meaningful line must declare nb_drones.
            if not nb_drones_parsed:
                nb_drones = self._parse_nb_drones(line, line_number)
                nb_drones_parsed = True
                continue

            if line.startswith("start_hub:"):
                if start_zone is not None:
                    raise ParseError(
                        line_number, "only one start_hub is allowed")
                start_zone = self._parse_zone_line(
                    line, "start_hub:", "start", line_number, nb_drones)
                self._register_zone(start_zone, zones, line_number)

            elif line.startswith("end_hub:"):
                if end_zone is not None:
                    raise ParseError(
                        line_number, "only one end_hub is allowed")
                end_zone = self._parse_zone_line(
                    line, "end_hub:", "end", line_number, nb_drones)
                self._register_zone(end_zone, zones, line_number)

            elif line.startswith("hub:"):
                zone = self._parse_zone_line(
                    line, "hub:", "hub", line_number)
                self._register_zone(zone, zones, line_number)

            elif line.startswith("connection:"):
                conn = self._parse_connection_line(
                    line, zones, seen_connections, line_number)
                connections.append(conn)

            else:
                raise ParseError(
                    line_number, f"unrecognised line prefix: {line!r}")

        if not nb_drones_parsed:
            raise ParseError(
                1, "file is empty or missing 'nb_drones:' declaration")
        if start_zone is None:
            raise ParseError(0, "missing start_hub declaration")
        if end_zone is None:
            raise ParseError(0, "missing end_hub declaration")
        if start_zone.max_drones < nb_drones:
            raise ParseError(
                0,
                f"start_hub max_drones ({start_zone.max_drones}) is less than "
                f"nb_drones ({nb_drones}) not all drones can occupy the start",
            )

        if end_zone.max_drones < nb_drones:
            raise ParseError(
                0,
                f"end_hub max_drones ({end_zone.max_drones}) is less than "
                f"nb_drones ({nb_drones}) — not all drones can reach the goal",
            )
        self._validate_reachability(zones, connections, start_zone, end_zone)

        return Graph(
            nb_drones=nb_drones,
            zones=zones,
            connections=connections,
            start=start_zone,
            end=end_zone,
        )

    def _read_lines(self) -> Generator[tuple[int, str], None, None]:
        """Yield (1-based line number, raw line) for every line in the file."""
        with open(self.file_path) as fh:
            for line_number, line in enumerate(fh, start=1):
                yield line_number, line

    def _register_zone(
        self, zone: Zone, zones: dict[str, Zone], line_number: int
    ) -> None:
        """Add *zone* to *zones*, raising ParseError on duplicate names."""
        if zone.name in zones:
            raise ParseError(
                line_number, f"duplicate zone name {zone.name!r}")
        zones[zone.name] = zone

    def _parse_nb_drones(self, line: str, line_number: int) -> int:
        """Parse ``nb_drones: <positive int>`` and return the integer value."""
        if not line.startswith("nb_drones:"):
            raise ParseError(
                line_number,
                f"expected 'nb_drones: <number>' as the first "
                f"declaration, got: {line!r}",
            )
        raw_value = line.split(":", 1)[1].strip()
        if not raw_value.isdigit() or int(raw_value) <= 0:
            raise ParseError(
                line_number,
                f"'nb_drones' must be a positive integer, "
                f"got {raw_value!r}",
            )
        return int(raw_value)

    def _parse_zone_line(
        self, line: str, prefix: str, role: str, line_number: int,
            default_max_drones: int = 1) -> Zone:
        """Parse a hub/start_hub/end_hub line and return a Zone."""
        value = line[len(prefix):].strip()
        # Split into at most 4 parts: name, x, y, optional metadata block.
        parts = value.split(maxsplit=3)

        if len(parts) < 3:
            raise ParseError(
                line_number,
                f"zone line requires at least name, x, y — got: {value!r}",
            )

        name: str = parts[0]
        self._validate_zone_name(name, line_number)

        try:
            x, y = int(parts[1]), int(parts[2])
        except ValueError:
            raise ParseError(
                line_number,
                f"zone coordinates must be integers, "
                f"got x={parts[1]!r} y={parts[2]!r}",
            ) from None

        raw_metadata = parts[3] if len(parts) > 3 else None
        zone_type, color, max_drones = self._parse_metadata(
            raw_metadata, line_number, default_max_drones)

        return Zone(
            name=name,
            coordinates=(x, y),
            zone_type=zone_type,
            color=color,
            max_drones=max_drones,
            role=role,
        )

    def _validate_zone_name(self, name: str, line_number: int) -> None:
        """Raise ParseError if *name* contains forbidden characters."""
        if "-" in name:
            raise ParseError(
                line_number,
                f"zone name {name!r} must not contain dashes "
                "(dashes are reserved as connection separators)",
            )
        if " " in name:
            raise ParseError(
                line_number,
                f"zone name {name!r} must not contain spaces",
            )

    def _parse_metadata(
        self, raw: Optional[str], line_number: int,
        default_max_drones: int = 1
    ) -> tuple[ZoneType, Optional[str], int]:
        """Parse an optional ``[key=value ...]`` metadata block.

        Returns zone_type, color, max_drones with defaults for omitted keys.
        """
        zone_type = ZoneType.NORMAL
        color: Optional[str] = None
        max_drones = default_max_drones

        if raw is None:
            return zone_type, color, max_drones

        if not (raw.startswith("[") and raw.endswith("]")):
            raise ParseError(
                line_number,
                f"metadata block must be wrapped in [ ], got: {raw!r}",
            )

        content = raw[1:-1].strip()
        if not content:
            return zone_type, color, max_drones

        valid_keys = {"zone", "color", "max_drones"}

        for token in content.split():
            if "=" not in token:
                raise ParseError(
                    line_number,
                    f"metadata token {token!r} is not in key=value format",
                )
            key, value = token.split("=", 1)

            if key not in valid_keys:
                raise ParseError(
                    line_number,
                    f"unknown metadata key {key!r}; "
                    f"allowed keys: {', '.join(sorted(valid_keys))}",
                )

            if key == "zone":
                try:
                    zone_type = ZoneType(value)
                except ValueError:
                    allowed = ", ".join(t.value for t in ZoneType)
                    raise ParseError(
                        line_number,
                        f"invalid zone type {value!r}; allowed: {allowed}",
                    ) from None

            elif key == "color":
                if not value:
                    raise ParseError(
                        line_number,
                        "color must be a non-empty word",
                    )
                color = value

            elif key == "max_drones":
                if not value.isdigit() or int(value) <= 0:
                    raise ParseError(
                        line_number,
                        f"max_drones must be a positive integer, "
                        f"got {value!r}",
                    )
                max_drones = int(value)

        return zone_type, color, max_drones

    def _parse_connection_line(
        self,
        line: str,
        zones: dict[str, Zone],
        seen: set[tuple[str, str]],
        line_number: int,
    ) -> Connection:
        """Parse a ``connection:`` line and return a Connection."""
        value = line.split(":", 1)[1].strip()
        parts = value.split(maxsplit=1)

        if not parts:
            raise ParseError(
                line_number,
                "connection line is empty after 'connection:'",
            )

        zone_pair = parts[0].split("-")
        if len(zone_pair) != 2 or not zone_pair[0] or not zone_pair[1]:
            raise ParseError(
                line_number,
                f"connection must be in zone1-zone2 format, "
                f"got {parts[0]!r}",
            )

        zone1_name, zone2_name = zone_pair

        # Self-loops are meaningless in this routing context.
        if zone1_name == zone2_name:
            raise ParseError(
                line_number,
                f"connection cannot link a zone to itself: {zone1_name!r}",
            )

        for name in (zone1_name, zone2_name):
            if name not in zones:
                raise ParseError(
                    line_number,
                    f"connection references undefined zone {name!r} "
                    "(zones must be declared before their connections)",
                )

        key = (min(zone1_name, zone2_name), max(zone1_name, zone2_name))
        if key in seen:
            raise ParseError(
                line_number,
                f"duplicate connection between "
                f"{zone1_name!r} and {zone2_name!r}",
            )
        seen.add(key)

        max_link_capacity = 1
        if len(parts) > 1:
            raw_meta = parts[1].strip()
            if not (raw_meta.startswith("[") and raw_meta.endswith("]")):
                raise ParseError(
                    line_number,
                    f"connection metadata must be wrapped in [ ], "
                    f"got {raw_meta!r}",
                )
            content = raw_meta[1:-1].strip()
            if not content.startswith("max_link_capacity="):
                raise ParseError(
                    line_number,
                    "only 'max_link_capacity' is valid in "
                    f"connection metadata, got {content!r}",
                )
            cap_str = content.split("=", 1)[1]
            if not cap_str.isdigit() or int(cap_str) <= 0:
                raise ParseError(
                    line_number,
                    f"max_link_capacity must be a positive integer, "
                    f"got {cap_str!r}",
                )
            max_link_capacity = int(cap_str)

        return Connection(zone1_name, zone2_name, max_link_capacity)

    def _validate_reachability(
        self,
        zones: dict[str, Zone],
        connections: list[Connection],
        start: Zone,
        end: Zone,
    ) -> None:
        """BFS from start — raise ParseError if end is not reachable.

        BLOCKED zones are treated as impassable and excluded from traversal.
        """
        adjacency: dict[str, set[str]] = {
            name: set()
            for name, zone in zones.items()
            if zone.zone_type is not ZoneType.BLOCKED
        }
        for conn in connections:
            z1, z2 = conn.zone1, conn.zone2
            if z1 in adjacency and z2 in adjacency:
                adjacency[z1].add(z2)
                adjacency[z2].add(z1)

        if start.name not in adjacency:
            raise ParseError(
                0, "start_hub is BLOCKED — no drone can leave it")
        if end.name not in adjacency:
            raise ParseError(
                0, "end_hub is BLOCKED — no drone can reach it")

        visited: set[str] = {start.name}
        queue: list[str] = [start.name]
        while queue:
            current = queue.pop(0)
            if current == end.name:
                return
            for neighbour in adjacency.get(current, set()):
                if neighbour not in visited:
                    visited.add(neighbour)
                    queue.append(neighbour)

        raise ParseError(
            0,
            f"no path exists between start_hub {start.name!r} "
            f"and end_hub {end.name!r}",
        )


def get_file_path() -> str:
    """Return the map file path from sys.argv, or exit with usage message."""
    if len(sys.argv) != 2:
        print("Usage: python3 main.py <map_file_path>")
        sys.exit(1)
    return sys.argv[1]
