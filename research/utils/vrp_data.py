from typing import NamedTuple


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


class InstanceData(NamedTuple):
    name: str
    num_vehicles: int
    vehicle_capacity: float
    velocity: float
    latest_due_date: int
    depots_data: list[VRPData]
    client_data: list[VRPData]

    @property
    def num_clients(self) -> int:
        return len(self.client_data)

    @property
    def num_depots(self) -> int:
        return len(self.depots_data)
