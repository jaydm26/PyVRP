# Either we can directly read the data which is in DIMACS format
from datetime import datetime
from collections import defaultdict
from itertools import product
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


from pyvrp import Edge, Model
from pyvrp._pyvrp import CongestionBehaviour, VelocityBehaviour
from pyvrp.plotting import plot_instance, plot_result, plot_route_schedule
from pyvrp.stop import MaxIterations, MaxRuntime
from pyvrp.stop.MultipleCriteria import MultipleCriteria
from research.utils.distance import get_distance_between_coordinates
from research.utils.duration import get_time_from_distance

EIGHT_HOURS = 0 * 60 * 60  # 8 hours in seconds


from instances.c101_C10 import instance_data as c101_C10
from instances.c205_C10 import instance_data as c205_C10
from instances.r102_C10 import instance_data as r102_C10
from instances.r201_C10 import instance_data as r201_C10
from instances.rc102_C10 import instance_data as rc102_C10
from instances.rc205_C10 import instance_data as rc205_C10

all_instances = [c101_C10, c205_C10, r102_C10, r201_C10, rc102_C10, rc205_C10]
velocity_behaviours = [
                VelocityBehaviour.ConstantVelocity,
                VelocityBehaviour.ConstantVelocityInSegment,
                VelocityBehaviour.VariableVelocity,
            ]
congestion_behaviours = [
                CongestionBehaviour.ConstantCongestion,
                CongestionBehaviour.ConstantCongestionInSegment,
                CongestionBehaviour.VariableCongestion,
            ]
multi_index = pd.MultiIndex.from_product(
    [
        list(product([str(vb) for vb in velocity_behaviours], [str(cb) for cb in congestion_behaviours])), 
        [
            "No. of Vehicles", 
            "Distance (in m)", 
            "Duration (in s)", 
            "Objective Function", 
            "Run Time (in s)", 
            "Routes"
        ]
    ], 
    names=["Case","Statistics"]
)
results_summary = pd.DataFrame(columns=[instance.name for instance in all_instances], index=multi_index)

for instance_data in all_instances:
    for velocity_behaviour in velocity_behaviours:
        for congestion_behaviour in congestion_behaviours:
            # REPEAT ALL OF THIS FOR EACH INSTANCE, VELOCITY BEHAVIOUR, AND CONGESTION BEHAVIOUR
            model = Model()

            model.velocity_behaviour = velocity_behaviour
            model.congestion_behaviour = congestion_behaviour

            depots = [
                model.add_depot(
                    x=depot.x,
                    y=depot.y,
                    # tw_early=EIGHT_HOURS,
                    # tw_late=instance_data.latest_due_date + EIGHT_HOURS,
                    name=depot.string_id,
                )
                for depot in [instance_data.depots_data[0]]
            ]

            clients = [
                model.add_client(
                    x=client.x,
                    y=client.y,
                    delivery=client.delivery_demand,
                    service_duration=client.service_time,
                    # tw_early=client.ready_time + EIGHT_HOURS,
                    # tw_late=client.due_date + EIGHT_HOURS,
                    # release_time=EIGHT_HOURS,
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
                    # tw_early=EIGHT_HOURS,
                    # tw_late=instance_data.latest_due_date + EIGHT_HOURS,
                    tw_late=16 * 3600,
                    vehicle_weight=instance_data.vehicle_weight,
                    power_to_mass_ratio=instance_data.power_to_mass_ratio,
                    min_hours_paid=6.0,
                    wage_per_hour=15.5,
                    unit_fuel_cost=0.55138,
                    unit_emission_cost=0.08532,
                    velocity=instance_data.velocity,
                    max_duration=10 * 3600,
                    unit_distance_cost=0.0002,
                    unit_duration_cost=0,
                    fixed_cost=217,
                )
                for depot in depots
            ]

            max_distance = max(
                [
                    get_distance_between_coordinates(from_node, to_node)
                    for from_node, to_node in product(model.locations, model.locations)
                ]
            )

            edges: list[Edge] = []
            for from_node, to_node in product(model.locations, model.locations):
                distance = get_distance_between_coordinates(from_node, to_node) * 1_000
                if model.data().velocity_behaviour == VelocityBehaviour.ConstantVelocity:
                    duration = round(distance / instance_data.velocity)
                else:
                    duration = get_time_from_distance(round(distance), round(max_distance))

                edges.append(
                    model.add_edge(
                        frm=from_node,
                        to=to_node,
                        distance=round(distance),
                        duration=duration,
                    )
                )

            result = model.solve(
                stop=MultipleCriteria([MaxIterations(500), MaxRuntime(1800)]),
            )

            print(result)

            file_name = Path(__file__).stem

            folder = Path(
                f"research/results/{file_name}_{instance_data.name}_{str(velocity_behaviour)}_{str(congestion_behaviour)}_{datetime.now().isoformat()}"
            )
            folder.mkdir(parents=True, exist_ok=True)

            fig_instance = plt.figure(figsize=(15, 9))
            plot_instance(model.data(), fig_instance)
            fig_instance.tight_layout()
            fig_instance.savefig(folder / "instance.png")
            plt.close(fig_instance)

            fig_result = plt.figure(figsize=(15, 9))
            plot_result(result, model.data(), fig_result)
            fig_result.tight_layout()
            fig_result.savefig(folder / "result.png")
            plt.close(fig_result)

            if len(result.best.routes()) > 1:
                fig_route_schedule, axs_route_schedule = plt.subplots(
                    len(result.best.routes()), 1, figsize=(15, 30)
                )
                for ax, route in zip(axs_route_schedule, result.best.routes(), strict=True):
                    plot_route_schedule(model.data(), route, ax=ax)
            else:
                fig_route_schedule, axs_route_schedule = plt.subplots(
                    1, 1, figsize=(15, 9)
                )
                plot_route_schedule(model.data(), result.best.routes()[0], ax=axs_route_schedule)

            fig_route_schedule.savefig(folder / "schedule.png")
            plt.close(fig_route_schedule)
            

            results_summary.loc[((str(velocity_behaviour), str(congestion_behaviour)), "No. of Vehicles"), instance_data.name] = result.best.num_routes()
            results_summary.loc[((str(velocity_behaviour), str(congestion_behaviour)), "Distance (in m)"), instance_data.name] = result.best.distance()
            results_summary.loc[((str(velocity_behaviour), str(congestion_behaviour)), "Duration (in s)"), instance_data.name] = result.best.duration()
            results_summary.loc[((str(velocity_behaviour), str(congestion_behaviour)), "Objective Function"), instance_data.name] = result.cost()
            results_summary.loc[((str(velocity_behaviour), str(congestion_behaviour)), "Run Time (in s)"), instance_data.name] = result.runtime
            results_summary.loc[((str(velocity_behaviour), str(congestion_behaviour)), "Routes"), instance_data.name] = "||".join(["D-" + "-".join([str(visit) for visit in trip.visits()]) + "-D" for route in result.best.routes() for trip in route.trips()])
            

results_summary.to_csv("test.csv", sep="\t")