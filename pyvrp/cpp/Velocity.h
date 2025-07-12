#pragma once
#ifndef PYVRP_VELOCITY_H
#define PYVRP_VELOCITY_H

#include "Measure.h"

#include <boost/math/interpolators/cardinal_cubic_b_spline.hpp>
#include <filesystem>
#include <vector>

namespace pyvrp::velocity
{

/**
 * Internal Enumeration to quickly ascertain the cost function to use
 */
enum VelocityBehaviour
{
    ConstantVelocity,
    ConstantVelocityInSegment,
    VariableVelocity,
};

class WLTCProfile
{
    std::string name_;
    std::filesystem::path path_;
    std::vector<double> time_;
    std::vector<double> velocity_;
    std::vector<double> squaredVelocity_;
    std::vector<double> cubedVelocity_;
    boost::math::interpolators::cardinal_cubic_b_spline<double> spline_;
    boost::math::interpolators::cardinal_cubic_b_spline<double> squaredSpline_;
    boost::math::interpolators::cardinal_cubic_b_spline<double> cubedSpline_;
    boost::math::interpolators::cardinal_cubic_b_spline<double> fullSpline_;
    double fullProfileDistance_;
    double repeatableProfileTime_;
    double repeatableProfileDistance_;
    double repeatableSquaredVelocityIntegral_;
    double repeatableCubedVelocityIntegral_;
    int startOffsetTime_;  // Left offset for the repeatable profile
    int endOffsetTime_;    // Right offset for the repeatable profile
    double startOffsetDistance_;
    double endOffsetDistance_;

public:
    WLTCProfile(const std::string &name,
                const std::filesystem::path &path,
                const int &startOffsetTime,
                const int &endOffsetTime);
    double getDistanceForTravelTime(double const &time) const;
    double getSquaredVelocityIntegral(double const &time) const;
    double getSquaredVelocityIntegral(Duration const &time) const;
    double getCubedVelocityIntegral(double const &time) const;
    double getCubedVelocityIntegral(Duration const &time) const;
    double getTimeForTravelDistance(double const &distance) const;

    std::string name() const { return name_; }
    std::string path() const { return path_; }
    std::vector<double> time() const { return time_; }
    std::vector<double> velocity() const { return velocity_; }
    boost::math::interpolators::cardinal_cubic_b_spline<double> const
    fullSpline()
    {
        return fullSpline_;
    }
    double fullProfileDistance() const { return fullProfileDistance_; }
    double repeatableProfileDistance() const
    {
        return repeatableProfileDistance_;
    }
    double repeatableProfileTime() const { return repeatableProfileTime_; }
    double startOffsetTime() const { return startOffsetTime_; }
    double endOffsetTime() const { return endOffsetTime_; }
    double startOffsetDistance() const { return startOffsetDistance_; }
    double endOffsetDistance() const { return endOffsetDistance_; }
    std::vector<double> splineTime() const
    {
        std::vector<double> vec(time_.begin() + startOffsetTime_,
                                time_.end() - endOffsetTime_);
        std::vector<double> outVec;
        std::transform(vec.begin(),
                       vec.end(),
                       std::back_inserter(outVec),
                       [vec](double x) { return x - vec.front(); });
        return outVec;
    }
    std::vector<double> splineVelocity() const
    {
        return std::vector<double>(velocity_.begin() + startOffsetTime_,
                                   velocity_.end() - endOffsetTime_);
    }

    boost::math::interpolators::cardinal_cubic_b_spline<double> spline() const
    {
        return spline_;
    };

    boost::math::interpolators::cardinal_cubic_b_spline<double>
    squaredSpline() const
    {
        return squaredSpline_;
    };

    boost::math::interpolators::cardinal_cubic_b_spline<double>
    cubedSpline() const
    {
        return cubedSpline_;
    };
};

// WLTCProfile slowVelocityProfile, mediumVelocityProfile, highVelocityProfile;

WLTCProfile getProfileBasedOnDistance(double const &distance);
WLTCProfile getProfileBasedOnDistance(Distance const &distance);
}  // namespace pyvrp::velocity

#endif  // PYVRP_VELOCITY_H
