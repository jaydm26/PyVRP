# Either we can directly read the data which is in DIMACS format
from datetime import datetime
from itertools import product
from pathlib import Path

import matplotlib.pyplot as plt

from instances.c101_C10 import instance_c101_C10 as instance_data
from pyvrp import Edge, Model
from pyvrp._pyvrp import CongestionBehaviour, VelocityBehaviour
from pyvrp.plotting import plot_instance, plot_result, plot_route_schedule
from pyvrp.stop import MaxIterations, MaxRuntime
from pyvrp.stop.MultipleCriteria import MultipleCriteria
from research.utils.distance import get_distance_between_coordinates
from research.utils.duration import get_time_from_distance

EIGHT_HOURS = 0 * 60 * 60  # 8 hours in seconds

model = Model()

model.velocity_behaviour = VelocityBehaviour.VariableVelocity
model.congestion_behaviour = CongestionBehaviour.VariableCongestion
depots = [
    model.add_depot(
        x=depot.x,
        y=depot.y,
        tw_early=EIGHT_HOURS,
        tw_late=instance_data.latest_due_date + EIGHT_HOURS,
        name=depot.string_id,
    )
    for depot in instance_data.depots_data
]

clients = [
    model.add_client(
        x=client.x,
        y=client.y,
        delivery=client.delivery_demand,
        service_duration=client.service_time,
        tw_early=client.ready_time + EIGHT_HOURS,
        tw_late=client.due_date + EIGHT_HOURS,
        release_time=EIGHT_HOURS,
        name=client.string_id,
    )
    for client in instance_data.client_data
]

vehicles = [
    model.add_vehicle_type(
        instance_data.num_vehicles,
        int(instance_data.vehicle_capacity),
        start_depot=depot,
        end_depot=depot,
        tw_early=EIGHT_HOURS,
        tw_late=instance_data.latest_due_date + EIGHT_HOURS,
        vehicle_weight=instance_data.vehicle_weight,
        power_to_mass_ratio=instance_data.power_to_mass_ratio,
        min_hours_paid=2.0,
        wage_per_hour=1.0,
        unit_fuel_cost=1000,
        unit_emission_cost=0,
        velocity=instance_data.velocity,
    )
    for depot in depots
]

edges: list[Edge] = []
for from_node, to_node in product(model.locations, model.locations):
    distance = get_distance_between_coordinates(from_node, to_node)
    duration = get_time_from_distance(int(distance))
    # duration = round(distance / velocity)
    edges.append(
        model.add_edge(
            frm=from_node,
            to=to_node,
            distance=int(distance),
            duration=duration,
        )
    )

result = model.solve(
    stop=MultipleCriteria([MaxIterations(10), MaxRuntime(1800)]),
)

print(result)

file_name = Path(__file__).stem

folder = Path(
    f"research/results/{file_name}_{instance_data.name}_{datetime.now().isoformat()}"
)
folder.mkdir(parents=True, exist_ok=True)

fig_instance = plt.figure(figsize=(15, 9))
plot_instance(model.data(), fig_instance)
fig_instance.tight_layout()
fig_instance.savefig(folder / "instance.png")

fig_result = plt.figure(figsize=(15, 9))
plot_result(result, model.data(), fig_result)
fig_result.tight_layout()
fig_result.savefig(folder / "resut.png")

fig_route_schedule, axs_route_schedule = plt.subplots(
    len(result.best.routes()), 1, figsize=(15, 30)
)

for ax, route in zip(axs_route_schedule, result.best.routes(), strict=True):
    plot_route_schedule(model.data(), route, ax=ax)

fig_route_schedule.savefig(folder / "schedule.png")
