from numpy.testing import assert_

from pyvrp import CostEvaluator, Solution
from pyvrp._pyvrp import ProblemData
from pyvrp.stop import FirstFeasible


def test_stops_on_first_feasible_solution(ok_small: ProblemData):
    """
    Tests that FirstFeasible stops when it first observes a feasible solution.
    """
    infeas = Solution(ok_small, [[1, 2, 3, 4]])
    assert_(not infeas.is_feasible())

    feas = Solution(ok_small, [[1, 2], [3, 4]])
    assert_(feas.is_feasible())

    stop = FirstFeasible()
    cost_eval = CostEvaluator([0], 0, 0, data=ok_small)
    assert_(not stop(cost_eval.cost(infeas)))
    assert_(stop(cost_eval.cost(feas)))
