from pathlib import Path

from research.utils.vrp_data import InstanceData, VRPData

data: list[VRPData] = [
  VRPData("S0", "f", 35, 35, 0, 0, 0, 0, 1000, 00),
  VRPData("S5", "f", 28, 62, 0, 0, 0, 0, 1000, 00),
  VRPData("S13", "f", 21, 22, 0, 0, 0, 0, 1000, 00),
  VRPData("S15", "f", 34, 16, 0, 0, 0, 0, 1000, 00),
  VRPData("C77", "c", 53, 43, 14, 3, 11, 86, 224, 100),
  VRPData("C50", "c", 47, 47, 13, 0, 13, 507, 599, 100),
  VRPData("C18", "c", 20, 40, 12, 6, 6, 403, 513, 100),
  VRPData("C28", "c", 41, 37, 16, 2, 14, 357, 495, 100),
  VRPData("C32", "c", 35, 69, 23, 12, 11, 501, 621, 100),
  VRPData("C31", "c", 31, 52, 27, 11, 16, 591, 809, 100),
  VRPData("C84", "c", 11, 31, 7, 5, 2, 314, 448, 100),
  VRPData("C72", "c", 47, 16, 25, 17, 8, 104, 252, 100),
  VRPData("C94", "c", 26, 27, 27, 1, 26, 534, 642, 100),
  VRPData("C100", "c", 18, 18, 17, 0, 17, 118, 148, 100),
]

num_vehicles = 1
vehicle_capacity = 1000
vehicle_weight = 1000.0
power_to_mass_ratio = 60 # Example value, adjust as needed
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
