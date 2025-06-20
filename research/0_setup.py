# Either we can directly read the data which is in DIMACS format
from datetime import datetime
from itertools import product
from pathlib import Path
from typing import NamedTuple

import matplotlib.pyplot as plt
import numpy as np

from pyvrp import Edge, Model
from pyvrp.PenaltyManager import PenaltyParams
from pyvrp._pyvrp import InternalCostBehaviour
from pyvrp.plotting import plot_instance, plot_result, plot_route_schedule
from pyvrp.solve import SolveParams
from pyvrp.stop import MaxIterations
from research.utils.distance import get_distance_between_coordinates
from research.utils.duration import get_time_from_distance

# instance = read("data/RC208_small.vrp")
# model = Model.from_data(instance)
#
# OR
# We specify the instance ourselves

model = Model()


class VRPData(NamedTuple):
    string_id: str
    type: str
    x: int
    y: int
    demand: int
    pickup_demand: int
    delivery_demand: int
    ready_time: int
    due_date: int
    service_time: int


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
vehicle_capacity = 200.0
velocity = 1.0
latest_due_date = max([data.due_date for data in c101_C10_data])

depots_data = list(filter(lambda x: x.type == "f", c101_C10_data))
client_data = list(filter(lambda x: x.type == "c", c101_C10_data))

depots = [
    model.add_depot(
        x=depot.x,
        y=depot.y,
        name=depot.string_id,
    )
    for depot in depots_data
]

clients = [
    model.add_client(
        x=client.x,
        y=client.y,
        delivery=client.delivery_demand,
        service_duration=client.service_time,
        tw_early=client.ready_time,
        tw_late=client.due_date,
        name=client.string_id,
    )
    for client in client_data
]

vehicles = [
    model.add_vehicle_type(
        num_vehicles,
        vehicle_capacity,
        start_depot=depot,
        end_depot=depot,
        tw_early=0,
        tw_late=latest_due_date,
    )
    for depot in depots
]

edges: list[Edge] = []
for from_node, to_node in product(model.locations, model.locations):
    distance = get_distance_between_coordinates(from_node, to_node)
    # duration = get_time_from_distance(distance)
    duration = round(distance / velocity)
    edges.append(
        model.add_edge(
            frm=from_node,
            to=to_node,
            distance=distance,
            duration=duration,
        )
    )

solve_params = SolveParams(
    penalty=PenaltyParams(
        velocity=velocity,
        cost_behaviour=InternalCostBehaviour.ConstantVelocityInSegmentWithConstantCongestion,
    )
)
result = model.solve(stop=MaxIterations(1000), params=solve_params)

print(result)

folder = Path(
    f"research/results/0_setup_c101_C10_{datetime.now().isoformat()}"
)
folder.mkdir(parents=True, exist_ok=True)

fig_instance = plt.figure(figsize=(15, 9))
plot_instance(model.data(), fig_instance)
fig_instance.tight_layout()
fig_instance.savefig(folder / "instance.png")

fig_result = plt.figure(figsize=(15, 9))
plot_result(result, model.data(), solve_params.penalty, fig_result)
fig_result.tight_layout()
fig_result.savefig(folder / "resut.png")

fig_route_schedule, axs_route_schedule = plt.subplots(
    len(result.best.routes()), 1, figsize=(15, 30)
)

for ax, route in zip(axs_route_schedule, result.best.routes(), strict=True):
    plot_route_schedule(model.data(), route, ax=ax)

fig_route_schedule.savefig(folder / "schedule.png")
