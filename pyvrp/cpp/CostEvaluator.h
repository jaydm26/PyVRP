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
    // { arg.route() } -> std::same_as<Route>;
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
    std::cout << "[penalisedCost] Evaluating penalised cost for proposal."
              << std::endl;
    Cost cost = 0;
    cost += arg.distanceCost();
    std::cout << "Added distance cost: " << arg.distanceCost() << std::endl;
    cost += arg.durationCost();
    std::cout << "Added duration cost: " << arg.durationCost() << std::endl;
    cost += !arg.empty() ? arg.fixedVehicleCost() : 0;
    std::cout << "Added fixed vehicle cost: "
              << (!arg.empty() ? arg.fixedVehicleCost() : 0) << std::endl;
    cost += excessLoadPenalties(arg.excessLoad());
    std::cout << "Added excess load penalties: "
              << excessLoadPenalties(arg.excessLoad()) << std::endl;
    cost += twPenalty(arg.timeWarp());
    std::cout << "Added time warp penalty: " << twPenalty(arg.timeWarp())
              << std::endl;
    cost += distPenalty(arg.excessDistance(), 0);
    std::cout << "Added distance penalty: "
              << distPenalty(arg.excessDistance(), 0) << std::endl;
    // auto cost = arg.distanceCost() + arg.durationCost()
    //             + (!arg.empty() ? arg.fixedVehicleCost() : 0)
    //             + excessLoadPenalties(arg.excessLoad())
    //             + twPenalty(arg.timeWarp())
    //             + distPenalty(arg.excessDistance(), 0);

    if constexpr (PrizeCostEvaluatable<T>)
        cost += arg.uncollectedPrizes();

    // cost += arg.wageCost(data_);

    // cost += arg.fuelAndEmissionCost(data_);

    return cost;
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
    std::cout << "[deltaCost (single proposal)] Evaluating delta cost for "
                 "proposal. Current delta: "
              << out << std::endl;
    auto const *route = proposal.route();

    out -= route->distanceCost();
    std::cout
        << "[deltaCost (single proposal)] After subtracting distance cost: "
        << out << std::endl;
    out -= distPenalty(route->distance(), route->maxDistance());
    std::cout
        << "[deltaCost (single proposal)] After subtracting distance penalty: "
        << out << std::endl;

    if constexpr (!skipLoad)
    {
        out -= excessLoadPenalties(route->excessLoad());
        std::cout << "[deltaCost (single proposal)] After subtracting excess "
                     "load penalty: "
                  << out << std::endl;
    }

    out -= route->durationCost();
    std::cout
        << "[deltaCost (single proposal)] After subtracting duration cost: "
        << out << std::endl;
    out -= twPenalty(route->timeWarp());
    std::cout
        << "[deltaCost (single proposal)] After subtracting time warp penalty: "
        << out << std::endl;

    // out -= route->wageCost(data_);

    // out -= route->fuelAndEmissionCost(data_);

    auto const distance = proposal.distance();
    out += route->unitDistanceCost() * static_cast<Cost>(distance);
    std::cout
        << "[deltaCost (single proposal)] After adding unit distance cost: "
        << out << std::endl;
    out += distPenalty(distance, route->maxDistance());
    std::cout << "[deltaCost (single proposal)] After adding distance penalty: "
              << out << std::endl;

    if constexpr (!exact)
        if (out >= 0)
            return false;

    if constexpr (!skipLoad)
    {
        auto const &capacity = route->capacity();
        for (size_t dim = 0; dim != capacity.size(); ++dim)
            out += loadPenalty(proposal.excessLoad(dim), 0, dim);
        std::cout
            << "[deltaCost (single proposal)] After adding load penalties: "
            << out << std::endl;
    }

    auto const [duration, timeWarp] = proposal.duration();
    out += route->unitDurationCost() * static_cast<Cost>(duration);
    std::cout
        << "[deltaCost (single proposal)] After adding unit duration cost: "
        << out << std::endl;
    out += twPenalty(timeWarp);
    std::cout
        << "[deltaCost (single proposal)] After adding time warp penalty: "
        << out << std::endl;

    // out += proposal.wageCost(data_);

    // out += proposal.fuelAndEmissionCost(data_);

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
    std::cout << "[deltaCost (two proposals)] Evaluating delta cost for "
                 "proposals. Current delta: "
              << out << std::endl;
    auto const *uRoute = uProposal.route();
    auto const *vRoute = vProposal.route();

    out -= uRoute->distanceCost();
    std::cout << "[deltaCost (two proposals)] After subtracting uRoute "
                 "distance cost: "
              << out << std::endl;
    out -= distPenalty(uRoute->distance(), uRoute->maxDistance());
    std::cout << "[deltaCost (two proposals)] After subtracting uRoute "
                 "distance penalty: "
              << out << std::endl;

    out -= vRoute->distanceCost();
    std::cout << "[deltaCost (two proposals)] After subtracting vRoute "
                 "distance cost: "
              << out << std::endl;
    out -= distPenalty(vRoute->distance(), vRoute->maxDistance());
    std::cout << "[deltaCost (two proposals)] After subtracting vRoute "
                 "distance penalty: "
              << out << std::endl;

    if constexpr (!skipLoad)
    {
        out -= excessLoadPenalties(uRoute->excessLoad());
        out -= excessLoadPenalties(vRoute->excessLoad());
    }

    out -= uRoute->durationCost();
    std::cout << "[deltaCost (two proposals)] After subtracting uRoute "
                 "duration cost: "
              << out << std::endl;
    out -= twPenalty(uRoute->timeWarp());
    std::cout << "[deltaCost (two proposals)] After subtracting uRoute "
                 "time warp penalty: "
              << out << std::endl;

    out -= vRoute->durationCost();
    std::cout << "[deltaCost (two proposals)] After subtracting vRoute "
                 "duration cost: "
              << out << std::endl;
    out -= twPenalty(vRoute->timeWarp());
    std::cout << "[deltaCost (two proposals)] After subtracting vRoute "
                 "time warp penalty: "
              << out << std::endl;

    // out -= uRoute->wageCost(data_);
    // out -= vRoute->wageCost(data_);

    // out -= uRoute->fuelAndEmissionCost(data_);
    // out -= vRoute->fuelAndEmissionCost(data_);

    auto const uDist = uProposal.distance();
    out += uRoute->unitDistanceCost() * static_cast<Cost>(uDist);
    std::cout << "[deltaCost (two proposals)] After adding uRoute unit "
                 "distance cost: "
              << out << std::endl;
    out += distPenalty(uDist, uRoute->maxDistance());
    std::cout << "[deltaCost (two proposals)] After adding uRoute distance "
                 "penalty: "
              << out << std::endl;

    auto const vDist = vProposal.distance();
    out += vRoute->unitDistanceCost() * static_cast<Cost>(vDist);
    std::cout << "[deltaCost (two proposals)] After adding vRoute unit "
                 "distance cost: "
              << out << std::endl;
    out += distPenalty(vDist, vRoute->maxDistance());
    std::cout << "[deltaCost (two proposals)] After adding vRoute distance "
                 "penalty: "
              << out << std::endl;

    if constexpr (!exact)
        if (out >= 0)
            return false;

    if constexpr (!skipLoad)
    {
        auto const &uCapacity = uRoute->capacity();
        for (size_t dim = 0; dim != uCapacity.size(); ++dim)
            out += loadPenalty(uProposal.excessLoad(dim), 0, dim);
        std::cout << "[deltaCost (two proposals)] After adding uRoute load "
                     "penalties: "
                  << out << std::endl;

        auto const &vCapacity = vRoute->capacity();
        for (size_t dim = 0; dim != vCapacity.size(); ++dim)
            out += loadPenalty(vProposal.excessLoad(dim), 0, dim);
        std::cout << "[deltaCost (two proposals)] After adding vRoute load "
                     "penalties: "
                  << out << std::endl;
    }

    if constexpr (!exact)
        if (out >= 0)
            return false;

    auto const [uDuration, uTimeWarp] = uProposal.duration();
    out += uRoute->unitDurationCost() * static_cast<Cost>(uDuration);
    std::cout << "[deltaCost (two proposals)] After adding uRoute unit "
                 "duration cost: "
              << out << std::endl;
    out += twPenalty(uTimeWarp);
    std::cout << "[deltaCost (two proposals)] After adding uRoute time warp "
                 "penalty: "
              << out << std::endl;
    // out += uProposal.wageCost(data_);
    // out += uProposal.fuelAndEmissionCost(data_);

    auto const [vDuration, vTimeWarp] = vProposal.duration();
    out += vRoute->unitDurationCost() * static_cast<Cost>(vDuration);
    std::cout << "[deltaCost (two proposals)] After adding vRoute unit "
                 "duration cost: "
              << out << std::endl;
    out += twPenalty(vTimeWarp);
    std::cout << "[deltaCost (two proposals)] After adding vRoute time warp "
                 "penalty: "
              << out << std::endl;
    // out += vProposal.wageCost(data_);
    // out += vProposal.fuelAndEmissionCost(data_);

    return true;
}
}  // namespace pyvrp

#endif  // PYVRP_COSTEVALUATOR_H
