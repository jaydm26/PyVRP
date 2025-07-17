#ifndef PYVRP_COSTEVALUATOR_H
#define PYVRP_COSTEVALUATOR_H

#include "Congestion.h"
#include "Measure.h"
#include "Solution.h"
#include "Velocity.h"
#include "search/Route.h"

#include <cassert>
#include <concepts>
#include <limits>
#include <utility>
#include <vector>

namespace pyvrp
{
// The following methods must be implemented for a type to be evaluatable by
// the CostEvaluator.
template <typename T>
concept CostEvaluatable = requires(T arg) {
    // { arg.route() } -> std::same_as<Route>;
    { arg.distanceCost() } -> std::same_as<Cost>;
    { arg.durationCost() } -> std::same_as<Cost>;
    { arg.fixedVehicleCost() } -> std::same_as<Cost>;
    { arg.excessLoad() } -> std::convertible_to<std::vector<Load>>;
    { arg.excessDistance() } -> std::same_as<Distance>;
    { arg.timeWarp() } -> std::same_as<Duration>;
    { arg.empty() } -> std::same_as<bool>;
    { arg.isFeasible() } -> std::same_as<bool>;
};

// an object that has the routes method available and maps to a vector of routes
template <typename T>
concept isSolution = std::is_same_v<std::remove_cvref_t<T>, Solution>;

template <typename T>
concept HasRoute = requires(T arg) {
    { arg.route() };
};

// If, additionally, methods related to optional clients and prize
// collecting are implemented we can also take that aspect into account. See
// the CostEvaluator implementation for details.
template <typename T>
concept PrizeCostEvaluatable = CostEvaluatable<T> && requires(T arg) {
    { arg.uncollectedPrizes() } -> std::same_as<Cost>;
};

// The following methods must be available before a type's delta cost can be
// evaluated by the CostEvaluator.
template <typename T>
concept DeltaCostEvaluatable = requires(T arg, size_t dimension) {
    { arg.route() };
    { arg.distance() } -> std::same_as<Distance>;
    { arg.duration() } -> std::convertible_to<std::pair<Duration, Duration>>;
    { arg.excessLoad(dimension) } -> std::same_as<Load>;
};

/**
 * CostEvaluator(
 *     load_penalties: list[float],
 *     tw_penalty: float,
 *     dist_penalty: float,
 * )
 *
 * Creates a CostEvaluator instance.
 *
 * This class stores various penalty terms, and can be used to determine the
 * costs of certain constraint violations.
 *
 * Parameters
 * ----------
 * load_penalties
 *    The penalty terms (one for each load dimension) for each unit of load in
 *    excess of the vehicle capacity.
 * tw_penalty
 *    The penalty for each unit of time warp.
 * dist_penalty
 *    The penalty for each unit of distance in excess of the vehicle's maximum
 *    distance constraint.
 *
 * Raises
 * ------
 * ValueError
 *     When any of the given penalty terms are negative.
 */
class CostEvaluator
{
    std::vector<double> loadPenalties_;  // per load dimension
    double twPenalty_;
    double distPenalty_;
    ProblemData data_;
    /**
     * Computes the cost penalty incurred from the given excess loads. This is
     * a convenient shorthand for calling ``loadPenalty`` for each dimension.
     */
    [[nodiscard]] inline Cost
    excessLoadPenalties(std::vector<Load> const &excessLoads) const;

public:
    CostEvaluator(std::vector<double> loadPenalties,
                  double twPenalty,
                  double distPenalty,
                  ProblemData data);

    /**
     * Computes the total excess load penalty for the given load and vehicle
     * capacity, and dimension.
     */
    [[nodiscard]] inline Cost
    loadPenalty(Load load, Load capacity, size_t dimension) const;

    /**
     * Computes the time warp penalty for the given time warp.
     */
    [[nodiscard]] inline Cost twPenalty(Duration timeWarp) const;

    /**
     * Computes the total excess distance penalty for the given distance.
     */
    [[nodiscard]] inline Cost distPenalty(Distance distance,
                                          Distance maxDistance) const;

    /**
     * Computes a smoothed objective (penalised cost) for a given solution.
     */
    // The docstring above is written for Python, where we only expose this
    // method for Solution.
    template <CostEvaluatable T>
    [[nodiscard]] Cost penalisedCost(T const &arg) const;

    /**
     * Hand-waving some details, each solution consists of a set of non-empty
     * routes :math:`\mathcal{R}`. Each route :math:`R \in \mathcal{R}` is a
     * sequence of edges, starting and ending at a depot. Each route :math:`R`
     * has an assigned vehicle type, through which the route is equipped with a
     * fixed vehicle cost :math:`f_R`, and unit distance and duration costs
     * :math:`c^\text{distance}_R` and :math:`c^\text{duration}_R`,
     * respectively. Let :math:`V_R = \{i : (i, j) \in R \}` be the set of
     * locations visited by route :math:`R`, and :math:`d_R` and :math:`t_R`
     * the total route distance and duration, respectively. The objective value
     * is then given by
     *
     * .. math::
     *
     *    \sum_{R \in \mathcal{R}}
     *      \left[
     *          f_R + c^\text{distance}_R d_R + c^\text{duration}_R t_R
     *      \right]
     *    + \sum_{i \in V} p_i - \sum_{R \in \mathcal{R}} \sum_{i \in V_R} p_i,
     *
     * where the first part lists each route's fixed, distance and duration
     * costs, respectively, and the second part the uncollected prizes of
     * unvisited clients.
     *
     * .. note::
     *
     *    The above cost computation only holds for feasible solutions. If the
     *    solution argument is *infeasible*, we return a very large number.
     *    If that is not what you want, consider calling :meth:`penalised_cost`
     *    instead.
     */
    // The docstring above is written for Python, where we only expose this
    // method for the Solution class.
    template <CostEvaluatable T> [[nodiscard]] Cost cost(T const &arg) const;

    /**
     * Evaluates the cost delta of the given route proposal, and writes the
     * resulting cost delta to the ``out`` parameter. The evaluation can be
     * exact, if the relevant template argument is set. Else it may shortcut
     * once it determines that the proposal does not constitute an improving
     * move. Optionally, several aspects of the evaluation may be skipped.
     *
     * The return value indicates whether the evaluation was exact or not.
     */
    template <bool exact = false,
              bool skipLoad = false,
              typename... Args,
              template <typename...> class T>
        requires(DeltaCostEvaluatable<T<Args...>>)
    bool deltaCost(Cost &out, T<Args...> const &proposal) const;

    /**
     * Evaluates the cost delta of the given route proposals, and writes the
     * resulting cost delta to the ``out`` parameter. The evaluation can be
     * exact, if the relevant template argument is set. Else it may shortcut
     * once it determines that the proposals do not constitute an improving
     * move. Optionally, several aspects of the evaluation may be skipped.
     *
     * The return value indicates whether the evaluation was exact or not.
     */
    template <bool exact = false,
              bool skipLoad = false,
              typename... uArgs,
              typename... vArgs,
              template <typename...> class T>
        requires(DeltaCostEvaluatable<T<uArgs...>>
                 && DeltaCostEvaluatable<T<vArgs...>>)
    bool deltaCost(Cost &out,
                   T<uArgs...> const &uProposal,
                   T<vArgs...> const &vProposal) const;
};

Cost CostEvaluator::excessLoadPenalties(
    std::vector<Load> const &excessLoads) const
{
    assert(excessLoads.size() == loadPenalties_.size());

    Cost cost = 0;
    for (size_t dim = 0; dim != loadPenalties_.size(); ++dim)
        cost += loadPenalties_[dim] * excessLoads[dim].get();

    return cost;
}

Cost CostEvaluator::loadPenalty(Load load,
                                Load capacity,
                                size_t dimension) const
{
    assert(dimension < loadPenalties_.size());
    auto const excessLoad = std::max<Load>(load - capacity, 0);
    return static_cast<Cost>(excessLoad.get() * loadPenalties_[dimension]);
}

Cost CostEvaluator::twPenalty([[maybe_unused]] Duration timeWarp) const
{
    return static_cast<Cost>(timeWarp.get() * twPenalty_);
}

Cost CostEvaluator::distPenalty(Distance distance, Distance maxDistance) const
{
    auto const excessDistance = std::max<Distance>(distance - maxDistance, 0);
    return static_cast<Cost>(excessDistance.get() * distPenalty_);
}

template <CostEvaluatable T>
Cost CostEvaluator::penalisedCost(T const &arg) const
{
    // Standard objective plus infeasibility-related penalty terms.
    std::cout << "Calculating penalised cost" << std::endl;  // IGNORE
    double cost = 0;

    std::cout << "Adding distance cost: " << arg.distanceCost()
              << std::endl;  // IGNORE
    cost += static_cast<double>(arg.distanceCost());

    std::cout << "Adding duration cost: " << arg.durationCost()
              << std::endl;  // IGNORE
    cost += static_cast<double>(arg.durationCost());

    std::cout << "Adding fixed vehicle cost: " << arg.fixedVehicleCost()
              << ". Is arg empty? " << arg.empty() << std::endl;  // IGNORE
    cost += static_cast<double>((!arg.empty() ? arg.fixedVehicleCost() : 0));

    std::cout << "Adding excess load penalties"
              << excessLoadPenalties(arg.excessLoad()) << std::endl;
    cost += static_cast<double>(excessLoadPenalties(arg.excessLoad()));

    std::cout << "Adding excess distance penalty: "
              << distPenalty(arg.excessDistance(), 0) << std::endl;
    cost += static_cast<double>(distPenalty(arg.excessDistance(), 0));

    std::cout << "Adding time warp penalty: " << twPenalty(arg.timeWarp())
              << std::endl;
    cost += static_cast<double>(twPenalty(arg.timeWarp()));

    if constexpr (PrizeCostEvaluatable<T>)
        cost += static_cast<double>(arg.uncollectedPrizes());

    std::cout << "Adding wage cost: " << arg.wageCost(data_) << std::endl;
    cost += arg.wageCost(data_);

    std::cout << "Adding fuel and emission cost: "
              << arg.fuelAndEmissionCost(data_) << std::endl;
    cost += arg.fuelAndEmissionCost(data_);

    return static_cast<Cost>(cost);
};

template <CostEvaluatable T> Cost CostEvaluator::cost(T const &arg) const
{
    // Penalties are zero when the solution is feasible, so we can fall
    // back to penalised cost in that case.
    return arg.isFeasible() ? penalisedCost(arg)
                            : std::numeric_limits<Cost>::max();
}

template <bool exact,
          bool skipLoad,
          typename... Args,
          template <typename...> class T>
    requires(DeltaCostEvaluatable<T<Args...>>)
bool CostEvaluator::deltaCost(Cost &out, T<Args...> const &proposal) const
{
    auto const *route = proposal.route();
    std::cout << "[Single Proposal] Delta Cost Before: " << out << std::endl;
    if (!route->empty())
    {
        std::cout
            << "[Single Proposal] Calculating delta cost for non-empty route"
            << std::endl;

        std::cout << "[Single Proposal][Route] Distance cost: "
                  << route->distanceCost() << std::endl;
        out -= route->distanceCost();

        std::cout << "[Single Proposal][Route] Distance Penalty: "
                  << distPenalty(route->distance(), route->maxDistance())
                  << std::endl;
        out -= distPenalty(route->distance(), route->maxDistance());

        std::cout << "[Single Proposal][Route] Excess load penalties("
                  << !skipLoad
                  << ") : " << excessLoadPenalties(route->excessLoad())
                  << std::endl;
        if constexpr (!skipLoad)
            out -= excessLoadPenalties(route->excessLoad());

        std::cout << "[Single Proposal][Route] Duration cost: "
                  << route->durationCost() << std::endl;
        out -= route->durationCost();
        std::cout << "[Single Proposal][Route] Time warp Penalty: "
                  << twPenalty(route->timeWarp()) << std::endl;
        out -= twPenalty(route->timeWarp());
        std::cout << "[Single Proposal][Route] Wage cost: "
                  << route->wageCost(data_) << std::endl;
        out -= route->wageCost(data_);

        std::cout << "[Single Proposal][Route] Fuel and emission cost: "
                  << route->fuelAndEmissionCost(data_) << std::endl;
        out -= route->fuelAndEmissionCost(data_);
    }

    std::cout << "[Single Proposal] Delta Cost After Subtractions: " << out
              << std::endl;

    auto const distance = proposal.distance();
    std::cout << "[Single Proposal][Proposal] Distance Cost: "
              << route->unitDistanceCost() * static_cast<Cost>(distance)
              << std::endl;
    out += route->unitDistanceCost() * static_cast<Cost>(distance);
    std::cout << "[Single Proposal][Proposal] Distance Penalty: "
              << distPenalty(distance, route->maxDistance()) << std::endl;
    out += distPenalty(distance, route->maxDistance());

    if constexpr (!skipLoad)
    {
        auto const &capacity = route->capacity();
        for (size_t dim = 0; dim != capacity.size(); ++dim)
        {
            if constexpr (!exact)
                if (out >= 0)
                    return false;
            std::cout << "[Single Proposal][Proposal] Excess Load Penalty for "
                         "dimension "
                      << dim << ": "
                      << loadPenalty(proposal.excessLoad(dim), 0, dim)
                      << std::endl;
            out += loadPenalty(proposal.excessLoad(dim), 0, dim);
        }
    }
    auto const [duration, timeWarp] = proposal.duration();
    std::cout << "[Single Proposal][Proposal] Duration Cost: "
              << route->unitDurationCost() * static_cast<Cost>(duration)
              << std::endl;
    out += route->unitDurationCost() * static_cast<Cost>(duration);
    std::cout << "[Single Proposal][Proposal] Time Warp Penalty: "
              << twPenalty(timeWarp) << std::endl;
    out += twPenalty(timeWarp);
    std::cout << "[Single Proposal][Proposal] Wage Cost: "
              << proposal.wageCost(data_) << std::endl;
    out += proposal.wageCost(data_);
    std::cout << "[Single Proposal][Proposal] Fuel and Emission Cost: "
              << proposal.fuelAndEmissionCost(data_) << std::endl;
    out += proposal.fuelAndEmissionCost(data_);

    return true;
}

template <bool exact,
          bool skipLoad,
          typename... uArgs,
          typename... vArgs,
          template <typename...> class T>
    requires(DeltaCostEvaluatable<T<uArgs...>>
             && DeltaCostEvaluatable<T<vArgs...>>)
bool CostEvaluator::deltaCost(Cost &out,
                              T<uArgs...> const &uProposal,
                              T<vArgs...> const &vProposal) const
{
    std::cout << "[Double Proposal] Delta Cost Before: " << out << std::endl;
    auto const *uRoute = uProposal.route();
    if (!uRoute->empty())
    {
        std::cout << "[Double Proposal] Calculating delta cost for uRoute"
                  << std::endl;
        std::cout << "[Double Proposal][uRoute] Distance cost: "
                  << uRoute->distanceCost() << std::endl;
        out -= uRoute->distanceCost();
        std::cout << "[Double Proposal][uRoute] Distance Penalty: "
                  << distPenalty(uRoute->distance(), uRoute->maxDistance())
                  << std::endl;
        out -= distPenalty(uRoute->distance(), uRoute->maxDistance());

        std::cout << "[Double Proposal][uRoute] Excess load penalties("
                  << !skipLoad
                  << "): " << excessLoadPenalties(uRoute->excessLoad())
                  << std::endl;
        if constexpr (!skipLoad)
            out -= excessLoadPenalties(uRoute->excessLoad());

        std::cout << "[Double Proposal][uRoute] Duration cost: "
                  << uRoute->durationCost() << std::endl;
        out -= uRoute->durationCost();
        std::cout << "[Double Proposal][uRoute] Time warp Penalty: "
                  << twPenalty(uRoute->timeWarp()) << std::endl;
        out -= twPenalty(uRoute->timeWarp());
        std::cout << "[Double Proposal][uRoute] Wage cost: "
                  << uRoute->wageCost(data_) << std::endl;
        out -= uRoute->wageCost(data_);
        std::cout << "[Double Proposal][uRoute] Fuel and emission cost: "
                  << uRoute->fuelAndEmissionCost(data_) << std::endl;
        out -= uRoute->fuelAndEmissionCost(data_);
    }

    std::cout << "[Double Proposal] Delta Cost After uRoute Subtractions: "
              << out << std::endl;

    auto const *vRoute = vProposal.route();
    if (!vRoute->empty())
    {
        std::cout << "[Double Proposal] Calculating delta cost for vRoute"
                  << std::endl;

        std::cout << "[Double Proposal][vRoute] Distance cost: "
                  << vRoute->distanceCost() << std::endl;
        out -= vRoute->distanceCost();
        std::cout << "[Double Proposal][vRoute] Distance Penalty: "
                  << distPenalty(vRoute->distance(), vRoute->maxDistance())
                  << std::endl;
        out -= distPenalty(vRoute->distance(), vRoute->maxDistance());

        std::cout << "[Double Proposal][vRoute] Excess load penalties("
                  << !skipLoad
                  << "): " << excessLoadPenalties(vRoute->excessLoad())
                  << std::endl;
        if constexpr (!skipLoad)
            out -= excessLoadPenalties(vRoute->excessLoad());

        std::cout << "[Double Proposal][vRoute] Duration cost: "
                  << vRoute->durationCost() << std::endl;
        out -= vRoute->durationCost();
        std::cout << "[Double Proposal][vRoute] Time warp Penalty: "
                  << twPenalty(vRoute->timeWarp()) << std::endl;
        out -= twPenalty(vRoute->timeWarp());
        std::cout << "[Double Proposal][vRoute] Wage cost: "
                  << vRoute->wageCost(data_) << std::endl;
        out -= vRoute->wageCost(data_);
        std::cout << "[Double Proposal][vRoute] Fuel and emission cost: "
                  << vRoute->fuelAndEmissionCost(data_) << std::endl;
        out -= vRoute->fuelAndEmissionCost(data_);
    }

    std::cout << "[Double Proposal] Delta Cost After vRoute Subtractions: "
              << out << std::endl;

    auto const uDist = uProposal.distance();
    std::cout << "[Double Proposal][uProposal] Distance Cost: "
              << uRoute->unitDistanceCost() * static_cast<Cost>(uDist)
              << std::endl;
    out += uRoute->unitDistanceCost() * static_cast<Cost>(uDist);
    std::cout << "[Double Proposal][uProposal] Distance Penalty: "
              << distPenalty(uDist, uRoute->maxDistance()) << std::endl;
    out += distPenalty(uDist, uRoute->maxDistance());

    auto const vDist = vProposal.distance();
    std::cout << "[Double Proposal][vProposal] Distance Cost: "
              << vRoute->unitDistanceCost() * static_cast<Cost>(vDist)
              << std::endl;
    out += vRoute->unitDistanceCost() * static_cast<Cost>(vDist);
    std::cout << "[Double Proposal][vProposal] Distance Penalty: "
              << distPenalty(vDist, vRoute->maxDistance()) << std::endl;
    out += distPenalty(vDist, vRoute->maxDistance());

    if constexpr (!skipLoad)
    {
        auto const &uCapacity = uRoute->capacity();
        for (size_t dim = 0; dim != uCapacity.size(); ++dim)
        {
            if constexpr (!exact)
                if (out >= 0)
                    return false;
            std::cout << "[Double Proposal][uProposal] Excess Load Penalty for "
                      << "dimension " << dim << ": "
                      << loadPenalty(uProposal.excessLoad(dim), 0, dim)
                      << std::endl;
            out += loadPenalty(uProposal.excessLoad(dim), 0, dim);
        }

        auto const &vCapacity = vRoute->capacity();
        for (size_t dim = 0; dim != vCapacity.size(); ++dim)
        {
            if constexpr (!exact)
                if (out >= 0)
                    return false;
            std::cout << "[Double Proposal][vProposal] Excess Load Penalty for "
                      << "dimension " << dim << ": "
                      << loadPenalty(vProposal.excessLoad(dim), 0, dim)
                      << std::endl;
            out += loadPenalty(vProposal.excessLoad(dim), 0, dim);
        }
    }

    if constexpr (!exact)
        if (out >= 0)
            return false;

    auto const [uDuration, uTimeWarp] = uProposal.duration();
    std::cout << "[Double Proposal][uProposal] Duration Cost: "
              << uRoute->unitDurationCost() * static_cast<Cost>(uDuration)
              << std::endl;
    out += uRoute->unitDurationCost() * static_cast<Cost>(uDuration);
    std::cout << "[Double Proposal][uProposal] Time Warp Penalty: "
              << twPenalty(uTimeWarp) << std::endl;
    out += twPenalty(uTimeWarp);
    std::cout << "[Double Proposal][uProposal] Wage Cost: "
              << uProposal.wageCost(data_) << std::endl;
    out += uProposal.wageCost(data_);
    std::cout << "[Double Proposal][uProposal] Fuel and Emission Cost: "
              << uProposal.fuelAndEmissionCost(data_) << std::endl;
    out += uProposal.fuelAndEmissionCost(data_);

    auto const [vDuration, vTimeWarp] = vProposal.duration();
    std::cout << "[Double Proposal][vProposal] Duration Cost: "
              << vRoute->unitDurationCost() * static_cast<Cost>(vDuration)
              << std::endl;
    out += vRoute->unitDurationCost() * static_cast<Cost>(vDuration);
    std::cout << "[Double Proposal][vProposal] Time Warp Penalty: "
              << twPenalty(vTimeWarp) << std::endl;
    out += twPenalty(vTimeWarp);
    std::cout << "[Double Proposal][vProposal] Wage Cost: "
              << vProposal.wageCost(data_) << std::endl;
    out += vProposal.wageCost(data_);
    std::cout << "[Double Proposal][vProposal] Fuel and Emission Cost: "
              << vProposal.fuelAndEmissionCost(data_) << std::endl;
    out += vProposal.fuelAndEmissionCost(data_);

    return true;
}
}  // namespace pyvrp

#endif  // PYVRP_COSTEVALUATOR_H
