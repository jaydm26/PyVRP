from pathlib import Path

from research.utils.vrp_data import InstanceData, VRPData

data: list[VRPData] = [
  VRPData("S0",	"f",	40,	50,	0,	0,	0,	0,	240,	0),
  VRPData("S1",	"f",	77,	52,	0,	0,	0,	0,	240,	0),
  VRPData("S15",	"f",	39,	26,	0,	0,	0,	0,	240,	0),
  VRPData("S19",	"f",	77,	30,	0,	0,	0,	0,	240,	0),
  VRPData("C70",	"c",	35,	69,	23,	12,	11,	83,	113,	10),
  VRPData("C57",	"c",	30,	25,	23,	4,	19,	27,	57,	10),
  VRPData("C49",	"c",	42,	12,	10,	8,	2,	85,	115,	10),
  VRPData("C45",	"c",	20,	82,	10,	8,	2,	53,	83,	10),
  VRPData("C54",	"c",	55,	60,	16,	2,	14,	170,	200,	10),
  VRPData("C92",	"c",	53,	43,	14,	3,	11,	140,	170,	10),
  VRPData("C26",	"c",	95,	30,	30,	21,	9,	121,	151,	10),
  VRPData("C11",	"c",	8,	40,	40,	32,	8,	86,	116,	10),
  VRPData("C53",	"c",	20,	50,	5,	3,	2,	0,	210,	10),
  VRPData("C44",	"c",	55,	82,	10,	4,	6,	36,	66,	10),
]

num_vehicles = 1
vehicle_capacity = 200
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
