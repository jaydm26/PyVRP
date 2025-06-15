#include "CostEvaluator.h"

#include <stdexcept>

using pyvrp::CostEvaluator;

CostEvaluator::CostEvaluator(std::vector<double> loadPenalties,
                             double twPenalty,
                             double distPenalty,
                             ProblemData data,
                             double unitFuelCost,
                             double unitEmissionCost,
                             double velocity,
                             double congestionFactor,
                             std::vector<std::vector<double>> fuelCosts,
                             double wagePerHour,
                             Duration minHoursPaid)
    : loadPenalties_(std::move(loadPenalties)),
      twPenalty_(twPenalty),
      distPenalty_(distPenalty),
      data_(data),
      unitFuelCost_(unitFuelCost),
      unitEmissionCost_(unitEmissionCost),
      velocity_(velocity),
      congestionFactor_(congestionFactor),
      fuelCosts_(std::move(fuelCosts)),
      wagePerHour_(wagePerHour),
      minHoursPaid_(minHoursPaid)
{
    for (auto const penalty : loadPenalties_)
        if (penalty < 0)
            throw std::invalid_argument("load_penalties must be >= 0.");

    if (twPenalty_ < 0)
        throw std::invalid_argument("tw_penalty must be >= 0.");

    if (distPenalty_ < 0)
        throw std::invalid_argument("dist_penalty must be >= 0.");
}
