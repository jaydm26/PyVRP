#pragma once
#ifndef PYVRP_CONGESTION_H
#define PYVRP_CONGESTION_H

#include "Measure.h"
#include <boost/math/interpolators/cardinal_cubic_b_spline.hpp>
#include <boost/math/quadrature/trapezoidal.hpp>
#include <boost/math/tools/roots.hpp>
#include <filesystem>
#include <vector>

namespace pyvrp::congestion
{

enum CongestionBehaviour
{
    ConstantCongestion,
    ConstantCongestionInSegment,
    VariableCongestion,  // Disabled for now. Will enable once needed.
};

class CongestionProfile
{
    std::string name_;
    std::filesystem::path path_;
    std::vector<double> time_;
    std::vector<double> congestion_;
    std::vector<double> squaredCongestion_;
    std::vector<double> cubedCongestion_;
    std::vector<double> quadrupledCongestion_;
    std::vector<double> pentupledCongestion_;
    std::vector<double> hextupledCongestion_;
    boost::math::interpolators::cardinal_cubic_b_spline<double> spline_;
    boost::math::interpolators::cardinal_cubic_b_spline<double> squaredSpline_;
    boost::math::interpolators::cardinal_cubic_b_spline<double> cubedSpline_;
    boost::math::interpolators::cardinal_cubic_b_spline<double>
        quadrupledSpline_;
    boost::math::interpolators::cardinal_cubic_b_spline<double>
        pentupledSpline_;
    boost::math::interpolators::cardinal_cubic_b_spline<double>
        hextupledSpline_;

public:
    CongestionProfile(const std::filesystem::path &path);

    std::string name() const { return name_; }
    std::string path() const { return path_; }
    std::vector<double> time() const { return time_; }
    std::vector<double> congestion() const { return congestion_; }

    double getCongestionIntegral(double const &from, double const &to) const;
    double getCongestionIntegral(Duration const &from,
                                 Duration const &to) const;
    double getSquaredCongestionIntegral(double const &from,
                                        double const &to) const;
    double getSquaredCongestionIntegral(Duration const &from,
                                        Duration const &to) const;
    double getCubedCongestionIntegral(double const &from,
                                      double const &time) const;
    double getCubedCongestionIntegral(Duration const &from,
                                      Duration const &to) const;

    double getDurationBasedOnDistanceAndVelocity(double const &distance,
                                                 double const &velocity,
                                                 double const &now) const;

    double getCongestionValue(const double &time) const
    {
        double gamma = spline_(time);
        // std::cout << "time: " << time << " | gamma: " << gamma
        //           << " | Congestion: "
        //           << (1 - 0.5 * gamma - 0.3 * gamma * gamma) << std::endl;
        auto const a = 1.0;
        auto const b = -0.5 * gamma;
        auto const c = -0.3 * gamma * gamma;

        return a + b + c;
    }

    double getCongestionValue(const pyvrp::Duration &time) const
    {
        return getCongestionValue(static_cast<double>(time));
    }

    double getSquaredCongestionValue(const double &time) const
    {
        double gamma = spline_(time);
        auto const a = 1.0;
        auto const b = -1 * gamma;
        auto const c = -0.35 * gamma * gamma;
        auto const d = 0.3 * gamma * gamma * gamma;
        auto const e = 0.09 * gamma * gamma * gamma * gamma;
        return a + b + c + d + e;
    }

    double getSquaredCongestionValue(const pyvrp::Duration &time) const
    {
        return getSquaredCongestionValue(static_cast<double>(time));
    }

    double getCubedCongestionValue(const double &time) const
    {
        double gamma = spline_(time);
        auto const a = 1.0;
        auto const b = -1.5 * gamma;
        auto const c = -0.15 * gamma * gamma;
        auto const d = 0.775 * gamma * gamma * gamma;
        auto const e = 0.045 * gamma * gamma * gamma * gamma;
        auto const f = -0.135 * gamma * gamma * gamma * gamma * gamma;
        auto const g = -0.027 * gamma * gamma * gamma * gamma * gamma * gamma;
        return a + b + c + d + e + f + g;
    }

    double getCubedCongestionValue(const pyvrp::Duration &time) const
    {
        return getCubedCongestionValue(static_cast<double>(time));
    }
};

CongestionProfile const getCongestionProfile(
    [[maybe_unused]] CongestionBehaviour const congestionBehaviour);
}  // namespace pyvrp::congestion

#endif  // PYVRP_CONGESTION_H
