"""Entry point for the drone delivery simulation.

Parses the map file, builds the routing graph, computes an optimal path for
each drone via Dijkstra, runs the turn-based simulation, and launches the
pygame visualiser.
"""

import sys
from file_parser import get_file_path, Parser, ParseError
from utils import AdjacencyList
from algorithm import Drone, Dijkstra, Simulation
from graphics import Game


if __name__ == "__main__":
    path = get_file_path()

    try:
        parsed_data = Parser(path).parse()
    except (ParseError, FileNotFoundError) as exc:
        print(f"Error: {exc}")
        sys.exit(1)

    list_drones: list[Drone] = []
    for i in range(1, parsed_data.nb_drones + 1):
        drone = Drone(f"D{i}", parsed_data.start.name)
        list_drones.append(drone)

    adj = AdjacencyList()

    for zone in parsed_data.zones.values():
        adj.add_zone(zone)

    for connection in parsed_data.connections:
        zone_a = parsed_data.zones[connection.zone1]
        zone_b = parsed_data.zones[connection.zone2]
        adj.add_connection(zone_a, zone_b, connection)

    dijkstra = Dijkstra(adj, parsed_data.zones)

    for drone in list_drones:
        drone_path = dijkstra.find_path(
            drone,
            drone.position,
            parsed_data.end.name,
            0
        )
        drone.path = drone_path

    simulation = Simulation(parsed_data, list_drones)
    simulation.run()

    game = Game()
    game.load_map(adj)
    game.load_drones(list_drones)
    game.run()
