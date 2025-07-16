from pyvrp import Model
from pyvrp.stop import MaxIterations

m = Model()

m.add_depot(
    x=0,
    y=0,
    # tw_late=25,
    name="Depot 1",
)
m.add_client(
    x=1,
    y=0,
    delivery=1,
    service_duration=10,
    # tw_early=1,
    name="Client 1",
)
m.add_client(
    x=2,
    y=0,
    delivery=1,
    service_duration=10,
    # tw_early=12,
    name="Client 2",
)
m.add_vehicle_type(
    num_available=1,
    capacity=100,
    start_depot=m.locations[0],
    end_depot=m.locations[0],
    # tw_early=0,
)

m.add_edge(frm=m.locations[0], to=m.locations[1], distance=1, duration=1)
m.add_edge(frm=m.locations[1], to=m.locations[0], distance=1, duration=1)
m.add_edge(frm=m.locations[1], to=m.locations[2], distance=1, duration=1)
m.add_edge(frm=m.locations[2], to=m.locations[1], distance=1, duration=1)
m.add_edge(frm=m.locations[0], to=m.locations[2], distance=2, duration=2)
m.add_edge(frm=m.locations[2], to=m.locations[0], distance=2, duration=2)

result = m.solve(stop=MaxIterations(100))
print(result)
