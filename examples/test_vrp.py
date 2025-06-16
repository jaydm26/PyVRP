from pathlib import Path
import matplotlib.pyplot as plt
from pyvrp import Model, read
from pyvrp.PenaltyManager import PenaltyParams
from pyvrp.solve import SolveParams
from pyvrp.stop import MaxIterations


RC208_FULL = "data/RC208.vrp"
RC208_SMALL = "data/RC208_small.vrp"
RC208_VERY_SMALL = "data/RC208_very_small.vrp"

CURRENT_DIR = Path(__file__).parent
instance = read(str(CURRENT_DIR / RC208_VERY_SMALL))
instance = instance.replace(
    vehicle_types=[
        instance.vehicle_type(0).replace(vehicle_weight=1, power_to_mass_ratio=1)
    ]
)
model = Model.from_data(instance)

params = SolveParams(
    penalty=PenaltyParams(
        unit_fuel_cost=1.0,
        unit_emission_cost=1.0,
        velocity=1,
        congestion_factor=1,
        wage_per_hour=1.0,
        min_hours_paid=8.0,
    )
)
result = model.solve(stop=MaxIterations(1), seed=42, display=True, params=params)
print(result)
