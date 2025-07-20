from pathlib import Path

from research.utils.vrp_data import InstanceData, VRPData

data: list[VRPData] = [
    VRPData("S0", "f", 40, 50, 0, 0, 0, 0, 3390, 0),
    VRPData("S3", "f", 57, 82, 0, 0, 0, 0, 3390, 0),
    VRPData("S15", "f", 39, 26, 0, 0, 0, 0, 3390, 0),
    VRPData("C8", "c", 34, 60, 20, 9, 11, 12, 332, 90),
    VRPData("C9", "c", 28, 70, 10, 6, 4, 138, 458, 90),
    VRPData("C47", "c", 30, 35, 10, 2, 8, 227, 547, 90),
    VRPData("C60", "c", 35, 5, 20, 18, 2, 1312, 1632, 90),
    VRPData("C94", "c", 65, 82, 10, 3, 7, 429, 749, 90),
    VRPData("C75", "c", 45, 65, 20, 7, 13, 1099, 1419, 90),
    VRPData("C66", "c", 47, 35, 10, 3, 7, 2633, 2953, 90),
    VRPData("C99", "c", 55, 80, 10, 4, 6, 810, 1130, 90),
    VRPData("C56", "c", 40, 5, 30, 27, 3, 1497, 1817, 90),
    VRPData("C15", "c", 20, 80, 40, 30, 10, 331, 651, 90),
]

num_vehicles = 1
vehicle_capacity = 700
vehicle_weight = 1000.0
power_to_mass_ratio = 60  # Example value, adjust as needed
velocity = 1.0
latest_due_date = max([data.due_date for data in data])

depots_data = list(filter(lambda x: x.type == "f", data))
client_data = list(filter(lambda x: x.type == "c", data))

instance_data = InstanceData(
    name=Path(__file__).stem,
    num_vehicles=num_vehicles,
    vehicle_capacity=vehicle_capacity,
    vehicle_weight=vehicle_weight,
    power_to_mass_ratio=power_to_mass_ratio,
    velocity=velocity,
    latest_due_date=latest_due_date,
    depots_data=depots_data,
    client_data=client_data,
)
