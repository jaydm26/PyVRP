#ifndef PYVRP_COSTEVALUATOR_H
#define PYVRP_COSTEVALUATOR_H

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

/**
 * Internal Enumeration to quickly ascertain the cost function to use
 */
enum INTERNAL_CostBehaviour
{
    ConstantVelocityWithConstantCongestion,
    ConstantVelocityWithConstantInSegmentCongestion,
    ConstantVelocityWithVariableCongestion,
    ConstantVelocityInSegmentWithConstantCongestion,
    ConstantVelocityInSegmentWithConstantInSegmentCongestion,
    ConstantVelocityInSegmentWithVariableCongestion,
    VariableVelocityWithConstantCongestion,
    VariableVelocityWithConstantInSegmentCongestion,
    VariableVelocityWithVariableCongestion,
};

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
    double unitFuelCost_;
    double unitEmissionCost_;
    double velocity_;
    double congestionFactor_;
    std::vector<std::vector<double>> fuelCosts_;
    double wagePerHour_;
    Duration minHoursPaid_;

    /**
     * Computes the cost penalty incurred from the given excess loads. This is
     * a convenient shorthand for calling ``loadPenalty`` for each dimension.
     */
    [[nodiscard]] inline Cost
    excessLoadPenalties(std::vector<Load> const &excessLoads) const;

private:
    /**
     * This variable controls how the cost is evaluated with regards to the fuel
     * and emission
     */
    INTERNAL_CostBehaviour costBehaviour_
        = INTERNAL_CostBehaviour::ConstantVelocityWithConstantCongestion;

public:
    CostEvaluator(std::vector<double> loadPenalties,
                  double twPenalty,
                  double distPenalty,
                  ProblemData data,
                  double unitFuelCost = 0.0,
                  double unitEmissionCost = 0.0,
                  double velocity = 0.0,
                  double congestionFactor = 1.0,
                  std::vector<std::vector<double>> fuelCosts = {},
                  double wagePerHour = 0.0,
                  Duration minHoursPaid = Duration(0));

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
     * Get the emission cost per ton per kilometer for a given vehicle's
     * power-to-mass ratio and velocity.
     */
    [[nodiscard]] inline double
    emissionCostPerTonPerHourConstantVelocity(double powerToMassRatio,
                                              double velocity) const;

    [[nodiscard]] inline double emissionCostPerTonPerHourNonLinearVelocity(
        double powerToMassRatio,
        double duration,
        double distance,
        double squaredVelocityIntegral,
        double cubedVelocityIntegral) const;

    /**
     * Compute the total fuel cost for the given distance and duration.
     */
    [[nodiscard]] inline Cost
    fuelAndEmissionCostWithConstantVelocityConstantCongestion(
        Route const &route) const;
    [[nodiscard]] inline Cost
    fuelAndEmissionCostWithConstantVelocityConstantCongestion(
        Solution const &solution) const;
    [[nodiscard]] inline Cost
    fuelAndEmissionCostWithConstantVelocityConstantCongestion(
        pyvrp::search::Route const &route) const;
    template <pyvrp::search::Segment... Segments>
    [[nodiscard]] inline Cost
    fuelAndEmissionCostWithConstantVelocityConstantCongestion(
        pyvrp::search::Route::Proposal<Segments...> const &proposal) const;

    [[nodiscard]] inline Cost
    fuelAndEmissionCostWithConstantVelocityInSegmentsConstantCongestion(
        Route const &route) const;
    [[nodiscard]] inline Cost
    fuelAndEmissionCostWithConstantVelocityInSegmentsConstantCongestion(
        Solution const &solution) const;
    [[nodiscard]] inline Cost
    fuelAndEmissionCostWithConstantVelocityInSegmentsConstantCongestion(
        pyvrp::search::Route const &route) const;
    template <pyvrp::search::Segment... Segments>
    [[nodiscard]] inline Cost
    fuelAndEmissionCostWithConstantVelocityInSegmentsConstantCongestion(
        pyvrp::search::Route::Proposal<Segments...> const &proposal) const;

    [[nodiscard]] inline Cost
    fuelAndEmissionCostWithNonLinearVelocityConstantCongestion(
        Route const &route) const;
    [[nodiscard]] inline Cost
    fuelAndEmissionCostWithNonLinearVelocityConstantCongestion(
        Solution const &solution) const;
    [[nodiscard]] inline Cost
    fuelAndEmissionCostWithNonLinearVelocityConstantCongestion(
        pyvrp::search::Route const &route) const;
    template <pyvrp::search::Segment... Segments>
    [[nodiscard]] inline Cost
    fuelAndEmissionCostWithNonLinearVelocityConstantCongestion(
        pyvrp::search::Route::Proposal<Segments...> const &proposal) const;

    /**
     * Computes the wage cost for the given hours worked, wage per hour, and
     * minimum hours paid.
     */
    [[nodiscard]] inline Cost wageCost(Duration hoursWorked,
                                       double wagePerHour,
                                       Duration minHoursPaid) const;

    template <typename T>
    [[nodiscard]] inline Cost applyFuelAndEmissionCost(T const &route) const;

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

// Do not attempt to modify velocity here. Consider this as a utility function
double CostEvaluator::emissionCostPerTonPerHourConstantVelocity(
    double powerToMassRatio, double velocity) const
{
    double a, b, c, d, e;
    a = 465.390;
    b = 48.143 * powerToMassRatio;
    c = (32.389 + 0.8931 * powerToMassRatio) * velocity;
    d = (-0.4771 - 0.02559 * powerToMassRatio) * velocity * velocity;
    e = (0.0008889 + 0.0004055 * powerToMassRatio) * velocity * velocity
        * velocity;

    return a + b + c + d + e;
}

double CostEvaluator::emissionCostPerTonPerHourNonLinearVelocity(
    double powerToMassRatio,
    double duration,
    double distance,
    double squaredVelocityIntegral,
    double cubedVelocityIntegral) const
{
    double a, b, c, d, e;
    a = 465.390 * duration;
    b = 48.143 * powerToMassRatio * duration;
    c = (32.389 + 0.8931 * powerToMassRatio) * distance;
    d = (-0.4771 - 0.02559 * powerToMassRatio) * squaredVelocityIntegral;
    e = (0.0008889 + 0.0004055 * powerToMassRatio) * cubedVelocityIntegral;

    return a + b + c + d + e;
}
/**
 * Calculate the fuel and emission cost where the velocity and congestion are a
 * constant term. Duration must be passed in hours, vehicle weight (inside
 * vehicleType) must be in tons, and the power-to-mass ratio shoud be in KW per
 * ton.
 */
Cost CostEvaluator::fuelAndEmissionCostWithConstantVelocityConstantCongestion(
    Route const &route) const
{
    double duration = route.duration().get();
    pyvrp::ProblemData::VehicleType vehicleType
        = data_.vehicleType(route.vehicleType());
    double emissionFactor
        = emissionCostPerTonPerHourConstantVelocity(
              vehicleType.powerToMassRatio, velocity_ * congestionFactor_)
          * vehicleType.vehicleWeight * duration;

    double fuelAndEmissionCost
        = (unitFuelCost_ + unitEmissionCost_) * emissionFactor;

    return static_cast<Cost>(fuelAndEmissionCost);
}

Cost CostEvaluator::fuelAndEmissionCostWithConstantVelocityConstantCongestion(
    Solution const &solution) const
{
    Cost cost = 0;
    for (Route route : solution.routes())
    {
        cost
            += fuelAndEmissionCostWithConstantVelocityConstantCongestion(route);
    }
    return cost;
}

Cost CostEvaluator::fuelAndEmissionCostWithConstantVelocityConstantCongestion(
    pyvrp::search::Route const &route) const
{
    double duration = route.duration().get();
    auto vehicleType = data_.vehicleType(route.vehicleType());
    double emissionFactor
        = emissionCostPerTonPerHourConstantVelocity(
              vehicleType.powerToMassRatio, velocity_ * congestionFactor_)
          * vehicleType.vehicleWeight * duration;
    double fuelAndEmissionCost
        = (unitFuelCost_ + unitEmissionCost_) * emissionFactor;

    return static_cast<Cost>(fuelAndEmissionCost);
}

template <pyvrp::search::Segment... Segments>
Cost CostEvaluator::fuelAndEmissionCostWithConstantVelocityConstantCongestion(
    pyvrp::search::Route::Proposal<Segments...> const &proposal) const
{
    auto const [duration, timeWarp] = proposal.duration();
    auto *route = proposal.route();
    auto vehicleType = data_.vehicleType(route->vehicleType());
    double emissionFactor
        = emissionCostPerTonPerHourConstantVelocity(
              vehicleType.powerToMassRatio, velocity_ * congestionFactor_)
          * vehicleType.vehicleWeight * duration.get();
    double fuelAndEmissionCost
        = (unitFuelCost_ + unitEmissionCost_) * emissionFactor;

    return static_cast<Cost>(fuelAndEmissionCost);
}

/**
 * Calculate fuel and emission cost for a route by breaking it down into
 * individual segments. For each segment, it is assumed that the velocity is
 * constant and it is calculated by getting the distance of the segment and the
 * duration to complete the segment.
 * Note that the route must be associated with a vehicle that has the vehicle
 * weight and power to mass ratio already defined. Otherwise, the value will be
 * 0.
 */
Cost CostEvaluator::
    fuelAndEmissionCostWithConstantVelocityInSegmentsConstantCongestion(
        Route const &route) const
{
    double costForRoute = 0;
    auto vehicleType = data_.vehicleType(route.vehicleType());
    auto distanceMatrix = data_.distanceMatrix(vehicleType.profile);
    auto durationMatrix = data_.durationMatrix(vehicleType.profile);

    for (Trip trip : route.trips())
    {
        size_t from = trip.startDepot();
        size_t to;

        for (size_t client : trip.visits())
        {
            size_t to = client;

            Distance distanceOfSegment = distanceMatrix(from, to);
            Duration durationOfSegment = durationMatrix(from, to);
            if (durationOfSegment == 0)
            {
                throw std::logic_error(
                    "Duration is 0. Velocity cannot be infinite. Did you "
                    "provide a duration matrix?");
            };
            double velocityInSegment
                = distanceOfSegment.get() / durationOfSegment.get();
            double emissionFactor = emissionCostPerTonPerHourConstantVelocity(
                                        vehicleType.powerToMassRatio,
                                        velocityInSegment * congestionFactor_)
                                    * vehicleType.vehicleWeight
                                    * durationOfSegment.get();

            double fuelAndEmissionCost
                = (unitFuelCost_ + unitEmissionCost_) * emissionFactor;

            costForRoute += fuelAndEmissionCost;
            from = to;
        }
        to = trip.endDepot();
        Distance distanceOfSegment = distanceMatrix(from, to);
        Duration durationOfSegment = durationMatrix(from, to);
        if (durationOfSegment == 0)
        {
            throw std::logic_error(
                "Duration is 0. Velocity cannot be infinite. Did you "
                "provide a duration matrix?");
        };
        double velocityInSegment
            = distanceOfSegment.get() / durationOfSegment.get();
        double emissionFactor = emissionCostPerTonPerHourConstantVelocity(
                                    vehicleType.powerToMassRatio,
                                    velocityInSegment * congestionFactor_)
                                * vehicleType.vehicleWeight
                                * durationOfSegment.get();

        double fuelAndEmissionCost
            = (unitFuelCost_ + unitEmissionCost_) * emissionFactor;

        costForRoute += fuelAndEmissionCost;
    }

    return static_cast<Cost>(costForRoute);
}

Cost CostEvaluator::
    fuelAndEmissionCostWithConstantVelocityInSegmentsConstantCongestion(
        Solution const &solution) const
{
    Cost cost = 0;
    for (Route route : solution.routes())
    {
        cost
            += fuelAndEmissionCostWithConstantVelocityInSegmentsConstantCongestion(
                route);
    }
    return cost;
}

Cost CostEvaluator::
    fuelAndEmissionCostWithConstantVelocityInSegmentsConstantCongestion(
        pyvrp::search::Route const &route) const
{
    auto vehicleType = data_.vehicleType(route.vehicleType());
    auto distanceMatrix = data_.distanceMatrix(vehicleType.profile);
    auto durationMatrix = data_.durationMatrix(vehicleType.profile);

    double cost = 0;

    size_t from, to;
    from = route.startDepot();
    for (size_t i = 1; i < route.size(); i++)
    {
        to = route[i]->client();
        Distance distanceOfSegment = distanceMatrix(from, to);
        Duration durationOfSegment = durationMatrix(from, to);
        if (durationOfSegment == 0)
        {
            throw std::logic_error(
                "Duration is 0. Velocity cannot be infinite. Did you "
                "provide a duration matrix?");
        };
        double velocityInSegment
            = distanceOfSegment.get() / durationOfSegment.get();
        double emissionFactor = emissionCostPerTonPerHourConstantVelocity(
                                    vehicleType.powerToMassRatio,
                                    velocityInSegment * congestionFactor_)
                                * vehicleType.vehicleWeight
                                * durationOfSegment.get();

        double fuelAndEmissionCost
            = (unitFuelCost_ + unitEmissionCost_) * emissionFactor;

        cost += fuelAndEmissionCost;
        from = to;
    }
    return static_cast<Cost>(cost);
}

template <pyvrp::search::Segment... Segments>
Cost CostEvaluator::
    fuelAndEmissionCostWithConstantVelocityInSegmentsConstantCongestion(
        pyvrp::search::Route::Proposal<Segments...> const &proposal) const
{
    auto segments = proposal.segments();
    auto impl = [&](auto... segment)
    {
        return (
            fuelAndEmissionCostWithConstantVelocityInSegmentsConstantCongestion(
                *segment.route())
            + ...);
    };

    Cost cost = std::apply(impl, segments);

    return cost;
}

Cost CostEvaluator::fuelAndEmissionCostWithNonLinearVelocityConstantCongestion(
    Route const &route) const
{
    double costForRoute = 0;
    auto vehicleType = data_.vehicleType(route.vehicleType());
    auto distanceMatrix = data_.distanceMatrix(vehicleType.profile);
    auto durationMatrix = data_.durationMatrix(vehicleType.profile);

    for (Trip trip : route.trips())
    {
        size_t from = trip.startDepot();
        size_t to;

        for (size_t client : trip.visits())
        {
            size_t to = client;
            Distance distanceOfSegment = distanceMatrix(from, to);
            Duration durationOfSegment = durationMatrix(from, to);
            if (durationOfSegment == 0)
            {
                throw std::logic_error(
                    "Duration is 0. Velocity cannot be infinite. Did you "
                    "provide a duration matrix?");
            };
            pyvrp::velocity::WLTCProfile velocityProfile
                = pyvrp::velocity::getProfileBasedOnDistance(
                    distanceOfSegment.get());
            double emissionFactor = emissionCostPerTonPerHourNonLinearVelocity(
                vehicleType.powerToMassRatio,
                durationOfSegment.get(),
                distanceOfSegment.get(),
                velocityProfile.getSquaredVelocityIntegral(
                    durationOfSegment.get()),
                velocityProfile.getCubedVelocityIntegral(
                    durationOfSegment.get()));

            double fuelAndEmissionCost
                = (unitFuelCost_ + unitEmissionCost_) * emissionFactor;

            costForRoute += fuelAndEmissionCost;
            from = to;
        }
        to = trip.endDepot();
        Distance distanceOfSegment = distanceMatrix(from, to);
        Duration durationOfSegment = durationMatrix(from, to);
        if (durationOfSegment == 0)
        {
            throw std::logic_error(
                "Duration is 0. Velocity cannot be infinite. Did you "
                "provide a duration matrix?");
        };
        pyvrp::velocity::WLTCProfile velocityProfile
            = pyvrp::velocity::getProfileBasedOnDistance(
                distanceOfSegment.get());
        double emissionFactor = emissionCostPerTonPerHourNonLinearVelocity(
            vehicleType.powerToMassRatio,
            durationOfSegment.get(),
            distanceOfSegment.get(),
            velocityProfile.getSquaredVelocityIntegral(durationOfSegment.get()),
            velocityProfile.getCubedVelocityIntegral(durationOfSegment.get()));

        double fuelAndEmissionCost
            = (unitFuelCost_ + unitEmissionCost_) * emissionFactor;

        costForRoute += fuelAndEmissionCost;
    }
    return static_cast<Cost>(costForRoute);
}

Cost CostEvaluator::fuelAndEmissionCostWithNonLinearVelocityConstantCongestion(
    Solution const &solution) const
{
    Cost cost = 0;
    for (Route route : solution.routes())
    {
        cost += fuelAndEmissionCostWithNonLinearVelocityConstantCongestion(
            route);
    }
    return cost;
}

Cost CostEvaluator::fuelAndEmissionCostWithNonLinearVelocityConstantCongestion(
    pyvrp::search::Route const &route) const
{
    double cost = 0;
    auto vehicleType = data_.vehicleType(route.vehicleType());
    auto distanceMatrix = data_.distanceMatrix(vehicleType.profile);
    auto durationMatrix = data_.durationMatrix(vehicleType.profile);

    size_t from, to;
    from = route.startDepot();
    for (size_t i = 1; i < route.size(); i++)
    {
        to = route[i]->client();
        Distance distanceOfSegment = distanceMatrix(from, to);
        Duration durationOfSegment = durationMatrix(from, to);
        if (durationOfSegment == 0)
        {
            throw std::logic_error(
                "Duration is 0. Velocity cannot be infinite. Did you "
                "provide a duration matrix?");
        };
        pyvrp::velocity::WLTCProfile velocityProfile
            = pyvrp::velocity::getProfileBasedOnDistance(
                distanceOfSegment.get());
        double emissionFactor = emissionCostPerTonPerHourNonLinearVelocity(
            vehicleType.powerToMassRatio,
            durationOfSegment.get(),
            distanceOfSegment.get(),
            velocityProfile.getSquaredVelocityIntegral(durationOfSegment.get()),
            velocityProfile.getCubedVelocityIntegral(durationOfSegment.get()));

        double fuelAndEmissionCost
            = (unitFuelCost_ + unitEmissionCost_) * emissionFactor;
        cost += fuelAndEmissionCost;
    }
    return static_cast<Cost>(cost);
}

template <pyvrp::search::Segment... Segments>
Cost CostEvaluator::fuelAndEmissionCostWithNonLinearVelocityConstantCongestion(
    pyvrp::search::Route::Proposal<Segments...> const &proposal) const
{
    std::tuple<Segments...> segments = proposal.segments();

    auto impl = [&](auto... segment)
    {
        return (fuelAndEmissionCostWithNonLinearVelocityConstantCongestion(
                    *segment.route())
                + ...);
    };

    Cost cost = std::apply(impl, segments);

    return cost;
}

Cost CostEvaluator::wageCost(Duration hoursWorked,
                             double wagePerHour,
                             Duration minHoursPaid) const
{
    // If the worked hours are less than the minimum paid hours, we pay
    // the minimum.
    auto const paidHours = std::max<Duration>(hoursWorked, minHoursPaid);
    return static_cast<Cost>(paidHours.get() * wagePerHour);
}

template <typename T>
Cost CostEvaluator::applyFuelAndEmissionCost(T const &route) const
{
    Cost cost = 0;
    if (costBehaviour_
        == INTERNAL_CostBehaviour::ConstantVelocityWithConstantCongestion)
    {
        cost
            += fuelAndEmissionCostWithConstantVelocityConstantCongestion(route);
    }
    else if (costBehaviour_
             == INTERNAL_CostBehaviour::
                 ConstantVelocityInSegmentWithConstantCongestion)
    {
        cost
            += fuelAndEmissionCostWithConstantVelocityInSegmentsConstantCongestion(
                route);
    }
    else if (costBehaviour_
             == INTERNAL_CostBehaviour::VariableVelocityWithConstantCongestion)
    {
        cost += fuelAndEmissionCostWithNonLinearVelocityConstantCongestion(
            route);
    }
    return cost;
}

template <CostEvaluatable T>
Cost CostEvaluator::penalisedCost(T const &arg) const
{
    // Standard objective plus infeasibility-related penalty terms.
    auto cost = arg.distanceCost() + arg.durationCost()
                + (!arg.empty() ? arg.fixedVehicleCost() : 0)
                + excessLoadPenalties(arg.excessLoad())
                + twPenalty(arg.timeWarp())
                + distPenalty(arg.excessDistance(), 0);

    if constexpr (PrizeCostEvaluatable<T>)
        cost += arg.uncollectedPrizes();

    cost += wageCost(std::ceil<Duration>(arg.duration().get() / 3600),
                     wagePerHour_,
                     minHoursPaid_);

    cost += applyFuelAndEmissionCost(arg);

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
    auto const *route = proposal.route();

    out -= route->distanceCost();
    out -= distPenalty(route->distance(), route->maxDistance());

    if constexpr (!skipLoad)
        out -= excessLoadPenalties(route->excessLoad());

    out -= route->durationCost();
    out -= twPenalty(route->timeWarp());

    auto const routeDuration = route->duration();
    out -= wageCost(std::ceil<Duration>(routeDuration.get() / 3600),
                    wagePerHour_,
                    minHoursPaid_);

    out -= applyFuelAndEmissionCost(*route);

    auto const distance = proposal.distance();
    out += route->unitDistanceCost() * static_cast<Cost>(distance);
    out += distPenalty(distance, route->maxDistance());

    if constexpr (!exact)
        if (out >= 0)
            return false;

    if constexpr (!skipLoad)
    {
        auto const &capacity = route->capacity();
        for (size_t dim = 0; dim != capacity.size(); ++dim)
            out += loadPenalty(proposal.excessLoad(dim), 0, dim);
    }

    auto const [duration, timeWarp] = proposal.duration();
    out += route->unitDurationCost() * static_cast<Cost>(duration);
    out += twPenalty(timeWarp);

    out += wageCost(std::ceil<Duration>(duration.get() / 3600),
                    wagePerHour_,
                    minHoursPaid_);

    out += applyFuelAndEmissionCost(proposal);

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
    auto const *uRoute = uProposal.route();
    auto const *vRoute = vProposal.route();

    out -= uRoute->distanceCost();
    out -= distPenalty(uRoute->distance(), uRoute->maxDistance());

    out -= vRoute->distanceCost();
    out -= distPenalty(vRoute->distance(), vRoute->maxDistance());

    if constexpr (!skipLoad)
    {
        out -= excessLoadPenalties(uRoute->excessLoad());
        out -= excessLoadPenalties(vRoute->excessLoad());
    }

    out -= uRoute->durationCost();
    out -= twPenalty(uRoute->timeWarp());

    out -= vRoute->durationCost();
    out -= twPenalty(vRoute->timeWarp());

    auto const uRouteDuration = uRoute->duration();
    out -= wageCost(std::ceil<Duration>(uRouteDuration.get() / 3600),
                    wagePerHour_,
                    minHoursPaid_);
    auto const vRouteDuration = vRoute->duration();
    out -= wageCost(std::ceil<Duration>(vRouteDuration.get() / 3600),
                    wagePerHour_,
                    minHoursPaid_);

    out -= applyFuelAndEmissionCost(*uRoute);
    out -= applyFuelAndEmissionCost(*vRoute);

    auto const uDist = uProposal.distance();
    out += uRoute->unitDistanceCost() * static_cast<Cost>(uDist);
    out += distPenalty(uDist, uRoute->maxDistance());

    auto const vDist = vProposal.distance();
    out += vRoute->unitDistanceCost() * static_cast<Cost>(vDist);
    out += distPenalty(vDist, vRoute->maxDistance());

    if constexpr (!exact)
        if (out >= 0)
            return false;

    if constexpr (!skipLoad)
    {
        auto const &uCapacity = uRoute->capacity();
        for (size_t dim = 0; dim != uCapacity.size(); ++dim)
            out += loadPenalty(uProposal.excessLoad(dim), 0, dim);

        auto const &vCapacity = vRoute->capacity();
        for (size_t dim = 0; dim != vCapacity.size(); ++dim)
            out += loadPenalty(vProposal.excessLoad(dim), 0, dim);
    }

    if constexpr (!exact)
        if (out >= 0)
            return false;

    auto const [uDuration, uTimeWarp] = uProposal.duration();
    out += uRoute->unitDurationCost() * static_cast<Cost>(uDuration);
    out += twPenalty(uTimeWarp);
    out += wageCost(std::ceil<Duration>(uDuration.get() / 3600),
                    wagePerHour_,
                    minHoursPaid_);

    out += applyFuelAndEmissionCost(uProposal);

    auto const [vDuration, vTimeWarp] = vProposal.duration();
    out += vRoute->unitDurationCost() * static_cast<Cost>(vDuration);
    out += twPenalty(vTimeWarp);
    out += wageCost(std::ceil<Duration>(vDuration.get() / 3600),
                    wagePerHour_,
                    minHoursPaid_);
    out += applyFuelAndEmissionCost(vProposal);

    return true;
}
}  // namespace pyvrp

#endif  // PYVRP_COSTEVALUATOR_H
