#pragma once
#ifndef PYVRP_CONGESTION_H
#define PYVRP_CONGESTION_H

#include "Measure.h"
#include <boost/math/interpolators/cardinal_cubic_b_spline.hpp>
#include <filesystem>
#include <vector>

namespace pyvrp::congestion
{

enum CongestionBehaviour
{
    ConstantCongestion,
    ConstantCongestionInSegment,
    // VariableCongestion, // Disabled for now. Will enable once needed.
};

class CongestionProfile
{
    std::string name_;
    std::filesystem::path const path_;
    std::vector<double> time_;
    std::vector<double> congestion_;
    boost::math::interpolators::cardinal_cubic_b_spline<double> spline_;

public:
    CongestionProfile(const std::filesystem::path &path);

    std::string name() const { return name_; }
    std::string path() const { return path_; }
    std::vector<double> time() const { return time_; }
    std::vector<double> congestion() const { return congestion_; }

    double getCongestionValue(const double &time) const
    {
        double gamma = spline_(time);
        // std::cout << "time: " << time << " | gamma: " << gamma
        //           << " | Congestion: "
        //           << (1 - 0.5 * gamma - 0.3 * gamma * gamma) << std::endl;

        return (1 - 0.5 * gamma - 0.3 * gamma * gamma);
    }
    double getCongestionValue(const pyvrp::Duration &time) const
    {
        return getCongestionValue(static_cast<double>(time));
    }
};

CongestionProfile const getCongestionProfile(
    [[maybe_unused]] CongestionBehaviour const congestionBehaviour);
}  // namespace pyvrp::congestion

#endif  // PYVRP_CONGESTION_H
