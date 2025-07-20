from research.utils.vrp_data import InstanceData, VRPData

c101_C10_data: list[VRPData] = [
    VRPData("S0", "f", 40, 50, 0, 0, 0, 0, 1236, 0),
    VRPData("S1", "f", 77, 52, 0, 0, 0, 0, 1236, 0),
    VRPData("S3", "f", 57, 82, 0, 0, 0, 0, 1236, 0),
    VRPData("S16", "f", 48, 8, 0, 0, 0, 0, 1236, 0),
    VRPData("S20", "f", 93, 43, 0, 0, 0, 0, 1236, 0),
    VRPData("C98", "c", 58, 75, 20, 5, 15, 181, 247, 90),
    VRPData("C78", "c", 88, 35, 20, 13, 7, 667, 731, 90),
    VRPData("C4", "c", 42, 68, 10, 4, 6, 584, 656, 90),
    VRPData("C13", "c", 22, 75, 30, 22, 8, 1042, 1106, 90),
    VRPData("C95", "c", 62, 80, 30, 7, 23, 274, 330, 90),
    VRPData("C100", "c", 55, 85, 20, 8, 12, 744, 798, 90),
    VRPData("C54", "c", 42, 10, 40, 31, 9, 810, 868, 90),
    VRPData("C27", "c", 23, 52, 10, 6, 4, 263, 311, 90),
    VRPData("C89", "c", 63, 58, 10, 1, 9, 929, 989, 90),
    VRPData("C96", "c", 60, 80, 10, 3, 7, 177, 243, 90),
]

num_vehicles = 4
vehicle_capacity = 1450 #instance uses capacity 200, taken assumption from previous MT
vehicle_weight = 3500.0
power_of_vehicle = 120
power_to_mass_ratio = power_of_vehicle/vehicle_weight  # Example value, adjust as needed
velocity = 10.0
latest_due_date = max([data.due_date for data in c101_C10_data])

depots_data = list(filter(lambda x: x.type == "f", c101_C10_data))
client_data = list(filter(lambda x: x.type == "c", c101_C10_data))

instance_data = InstanceData(
    name="c101_C10",
    num_vehicles=num_vehicles,
    vehicle_capacity=vehicle_capacity,
    vehicle_weight=vehicle_weight,
    power_to_mass_ratio=power_to_mass_ratio,
    velocity=velocity,
    latest_due_date=latest_due_date,
    depots_data=depots_data,
    client_data=client_data,
)
