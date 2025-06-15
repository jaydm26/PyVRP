import math
from dataclasses import dataclass

from pyvrp.PenaltyManager import PenaltyParams
from pyvrp.Statistics import Statistics
from pyvrp._pyvrp import CostEvaluator, ProblemData, Solution


@dataclass
class Result:
    """
    Stores the outcomes of a single run. An instance of this class is returned
    once the GeneticAlgorithm completes.

    Parameters
    ----------
    best
        The best observed solution.
    stats
        A Statistics object containing runtime statistics.
    num_iterations
        Number of iterations performed by the genetic algorithm.
    runtime
        Total runtime of the main genetic algorithm loop.

    Raises
    ------
    ValueError
        When the number of iterations or runtime are negative.
    """

    best: Solution
    stats: Statistics
    num_iterations: int
    runtime: float

    def __post_init__(self):
        if self.num_iterations < 0:
            raise ValueError("Negative number of iterations not understood.")

        if self.runtime < 0:
            raise ValueError("Negative runtime not understood.")

    def cost(
        self,
        data: ProblemData,
        penalty_params: PenaltyParams = PenaltyParams(),
    ) -> float:
        """
        Returns the cost (objective) value of the best solution. Returns inf
        if the best solution is infeasible.
        """
        if not self.best.is_feasible():
            return math.inf

        num_load_dims = len(self.best.excess_load())
        return CostEvaluator(
            [0] * num_load_dims,
            0,
            0,
            data,
            unit_fuel_cost=penalty_params.unit_fuel_cost,
            unit_emission_cost=penalty_params.unit_emission_cost,
            velocity=penalty_params.velocity,
            congestion_factor=penalty_params.congestion_factor,
            fuel_costs=penalty_params.fuel_costs,
            wage_per_hour=penalty_params.wage_per_hour,
            min_hours_paid=penalty_params.wage_per_hour,
        ).cost(self.best)

    def is_feasible(self) -> bool:
        """
        Returns whether the best solution is feasible.
        """
        return self.best.is_feasible()

    def summary(
        self,
        data: ProblemData,
        penalty_params: PenaltyParams = PenaltyParams(),
    ) -> str:
        """
        Returns a nicely formatted result summary.
        """
        obj_str = (
            f"{self.cost(data, penalty_params)}" if self.is_feasible() else "INFEASIBLE"
        )
        summary = [
            "Solution results",
            "================",
            f"    # routes: {self.best.num_routes()}",
            f"     # trips: {self.best.num_trips()}",
            f"   # clients: {self.best.num_clients()}",
            f"   objective: {obj_str}",
            f"    distance: {self.best.distance()}",
            f"    duration: {self.best.duration()}",
            f"# iterations: {self.num_iterations}",
            f"    run-time: {self.runtime:.2f} seconds",
        ]

        return "\n".join(summary)

    def __str__(self) -> str:
        content = [
            # self.summary(),
            "",
            "Routes",
            "------",
            str(self.best),
        ]

        return "\n".join(content)
