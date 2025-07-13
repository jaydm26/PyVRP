#include "CongestedVelocity.h"
#include <boost/math/quadrature/trapezoidal.hpp>
#include <boost/math/tools/roots.hpp>
#include <cassert>

double pyvrp::congestedVelocity::CongestedWLTCProfile::
    getDistanceForCongestedTravelTime(double const &from,
                                      double const &to) const
{

    auto splineTime_ = wltcProfile_.splineTime();
    auto congestedVelocity = [&](double x)
    {
        double remainingTime = std::fmod(x - from, splineTime_.back());
        return wltcProfile_.spline()(remainingTime)
               * congestionProfile_.getCongestionValue(x);
    };

    return boost::math::quadrature::trapezoidal(congestedVelocity, from, to);
}

double pyvrp::congestedVelocity::CongestedWLTCProfile::
    getSquaredVelocityIntegralForCongestedTravelTime(double const &from,
                                                     double const &to) const
{

    auto splineTime_ = wltcProfile_.splineTime();
    auto congestedSquaredVelocity = [&](double x)
    {
        double remainingTime = std::fmod(x - from, splineTime_.back());
        return wltcProfile_.squaredSpline()(remainingTime)
               * std::pow(congestionProfile_.getCongestionValue(x), 2);
    };

    return boost::math::quadrature::trapezoidal(
        congestedSquaredVelocity, from, to);
}

double pyvrp::congestedVelocity::CongestedWLTCProfile::
    getCubedVelocityIntegralForCongestedTravelTime(double const &from,
                                                   double const &to) const
{

    auto splineTime_ = wltcProfile_.splineTime();
    auto congestedCubedVelocity = [&](double x)
    {
        double remainingTime = std::fmod(x - from, splineTime_.back());
        return wltcProfile_.cubedSpline()(remainingTime)
               * std::pow(congestionProfile_.getCongestionValue(x), 3);
    };

    return boost::math::quadrature::trapezoidal(
        congestedCubedVelocity, from, to);
}

double pyvrp::congestedVelocity::CongestedWLTCProfile::
    getCongestedTravelTimeForDistance(double const &distance,
                                      double const &from) const
{

    double coveredDistance = 0.0;
    double elapsedTime = 0.0;
    auto splineTime_ = wltcProfile_.splineTime();
    while (distance > coveredDistance)
    {
        auto congestedVelocity = [&](double x)
        {
            double remainingTime = std::fmod(x - from, splineTime_.back());
            return wltcProfile_.spline()(remainingTime)
                   * congestionProfile_.getCongestionValue(x);
        };
        assert(splineTime_.size() > 0);
        double fullSplineDistance
            = boost::math::quadrature::trapezoidal(congestedVelocity,
                                                   splineTime_.front(),
                                                   splineTime_.back());  // in m
        if (coveredDistance + fullSplineDistance > distance)
        {
            auto remainingDistance = distance - coveredDistance;  // in m
            auto func = [&](double t)
            {
                return boost::math::quadrature::trapezoidal(
                           congestedVelocity, elapsedTime, t)
                       - remainingDistance;
            };

            assert(wltcProfile_.time().size() > 0);
            auto result = boost::math::tools::bisect(
                func,
                wltcProfile_.time().front(),
                wltcProfile_.time().back(),
                [](double a, double b) { return std::abs(a - b) <= 1e-8; });

            elapsedTime += (result.first + result.second) / 2;  // in s
            break;  // We have found the time for the distance.
        }
        else
        {
            coveredDistance += fullSplineDistance;  // in m
            elapsedTime += splineTime_.back();      // in s
        }
    }

    return elapsedTime;
}