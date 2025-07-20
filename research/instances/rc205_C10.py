from pathlib import Path

from research.utils.vrp_data import InstanceData, VRPData

data: list[VRPData] = [
  VRPData("S0",	"f",	40,	50,	0,	0,	0,	0,	960,	0),
  VRPData("S1",	"f",	77,	52,	0,	0,	0,	0,	960,	0),
  VRPData("S3",	"f",	57,	82,	0,	0,	0,	0,	960,	0),
  VRPData("S11",	"f",	10,	28,	0,	0,	0,	0,	960,	0),
  VRPData("C96",	"c",	55,	54,	26,	1,	25,	319,	355,	10),
  VRPData("C43",	"c",	55,	85,	20,	8,	12,	39,	490,	10),
  VRPData("C87",	"c",	12,	24,	13,	7,	6,	441,	441,	10),
  VRPData("C100",	"c",	31,	67,	3,	2,	1,	528,	528,	10),
  VRPData("C15",	"c",	2,	40,	20,	19,	1,	607,	739,	10),
  VRPData("C58",	"c",	15,	10,	20,	7,	13,	198,	406,	10),
  VRPData("C31",	"c",	88,	35,	20,	13,	7,	71,	689,	10),
  VRPData("C82",	"c",	27,	43,	9,	4,	5,	488,	650,	10),
  VRPData("C93",	"c",	61,	52,	3,	1,	2,	350,	404,	10),
  VRPData("C6",	"c",	18,	75,	20,	16,	4,	142,	494,	10),
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
