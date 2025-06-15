from pathlib import Path
import matplotlib.pyplot as plt
from pyvrp import Model, read
from pyvrp.PenaltyManager import PenaltyParams
from pyvrp.solve import SolveParams
from pyvrp.stop import MaxIterations


CURRENT_DIR = Path(__file__).parent
instance = read(str(CURRENT_DIR / "data/RC208_small.vrp"))
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
