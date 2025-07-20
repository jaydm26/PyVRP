from pathlib import Path

from research.utils.vrp_data import InstanceData, VRPData

data: list[VRPData] = [
    VRPData("S0", "f", 35, 35, 0, 0, 0, 0, 230, 0),
    VRPData("S5", "f", 28, 62, 0, 0, 0, 0, 230, 0),
    VRPData("S17", "f", 51, 7, 0, 0, 0, 0, 230, 0),
    VRPData("S18", "f", 63, 12, 0, 0, 0, 0, 230, 0),
    VRPData("C31", "c", 31, 52, 27, 11, 16, 25, 35, 10),
    VRPData("C23", "c", 55, 5, 29, 27, 2, 97, 107, 10),
    VRPData("C67", "c", 67, 5, 25, 24, 1, 0, 176, 10),
    VRPData("C60", "c", 17, 34, 3, 2, 1, 0, 201, 10),
    VRPData("C88", "c", 26, 52, 9, 5, 4, 166, 176, 10),
    VRPData("C77", "c", 53, 43, 14, 3, 11, 150, 160, 10),
    VRPData("C20", "c", 45, 65, 9, 3, 6, 77, 87, 10),
    VRPData("C99", "c", 20, 26, 9, 3, 6, 138, 148, 10),
    VRPData("C12", "c", 50, 35, 19, 6, 13, 177, 187, 10),
    VRPData("C21", "c", 45, 20, 11, 7, 4, 0, 201, 10),
]

num_vehicles = 1
vehicle_capacity = 200
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
