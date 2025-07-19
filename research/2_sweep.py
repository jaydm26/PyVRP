import math

from research.instances.c101_C10 import instance_c101_C10 as instance_data
from research.utils.vrp_data import VRPData


def polar_angle(client: VRPData, depot: VRPData) -> float:
    """Compute polar angle relative to the depot."""
    dx = client.x - depot.x
    dy = client.y - depot.y
    return math.atan2(dy, dx)


def distance(a: VRPData, b: VRPData) -> float:
    """Euclidean distance between two points a and b."""
    return math.hypot(a.x - b.x, a.y - b.y)


def nearest_neighbor_route(
    clients: list[VRPData], depot: VRPData
) -> list[VRPData]:
    """
    Simple nearest neighbor TSP heuristic for a given set of customers.
    Returns the route starting and ending at the depot.
    """
    if not clients:
        return []

    route = [clients[0]]
    visited = {clients[0].string_id}
    current = clients[0]

    while len(route) < len(clients):
        next_client = min(
            (c for c in clients if c.string_id not in visited),
            key=lambda c: distance(current, c),
        )
        route.append(next_client)
        visited.add(next_client.string_id)
        current = next_client

    return route


def sweep_vrp(clients: list[VRPData], depot: VRPData, vehicle_capacity: float):
    """
    Sweep algorithm for VRP:
    1. Convert customers to polar angles.
    2. Sort by angle.
    3. Sweep and assign to routes.
    4. Optimize each route with nearest neighbor.
    """
    # Step 1: Compute angles
    client_angles = {c: polar_angle(c, depot) for c in clients}

    # Step 2: Sort customers by angle
    sorted_client_angles = sorted(client_angles.items(), key=lambda x: x[1])

    clients = [c for c, _ in sorted_client_angles]

    # Step 3: Sweep
    routes: list[list[VRPData]] = []
    current_route: list[VRPData] = []
    current_load = 0.0

    for c in clients:
        if current_load + c.demand > vehicle_capacity:
            # Optimize current route
            optimized_route = nearest_neighbor_route(current_route, depot)
            routes.append(optimized_route)
            current_route = []
            current_load = 0.0
        current_route.append(c)
        current_load += c.demand

    if current_route:
        optimized_route = nearest_neighbor_route(current_route, depot)
        routes.append(optimized_route)

    return routes


def route_distance(route: list[VRPData], depot: VRPData) -> float:
    """Compute total distance of a route starting/ending at depot."""
    if not route:
        return 0.0
    total = distance(depot, route[0])
    for i in range(len(route) - 1):
        total += distance(route[i], route[i + 1])
    total += distance(route[-1], depot)
    return total


# Example usage
clients = instance_data.client_data
depot = instance_data.depots_data[0]  # Assuming one depot
vehicle_capacity = instance_data.vehicle_capacity
routes = sweep_vrp(clients, depot, vehicle_capacity)

total_distance = 0.0
for idx, route in enumerate(routes):
    route_ids = [c.string_id for c in route]
    dist = route_distance(route, depot)
    total_distance += dist
    print(f"Route {idx + 1}: {route_ids} | Distance: {dist:.2f}")

print(f"Total Distance: {total_distance:.2f}")
