def service_time_for_vehicle_to_unload(
    demand: float,
    small_vehicle_unload_time: float,
    large_vehicle_unload_time: float,
    small_vehicle_capacity: float,
    large_vehicle_capacity: float,
) -> float:
    """
    Service time for a vehicle to unload at a client.
    This is a constant value of 1.0 for all clients.
    """
    σ_2 = (large_vehicle_unload_time - small_vehicle_unload_time) / (
        large_vehicle_capacity - small_vehicle_capacity
    )
    σ_1 = small_vehicle_unload_time - σ_2 * small_vehicle_capacity
    service_time = σ_1 + σ_2 * demand

    return service_time
