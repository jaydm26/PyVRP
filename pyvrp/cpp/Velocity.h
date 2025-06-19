#pragma once
#ifndef PYVRP_VELOCITY_H
#define PYVRP_VELOCITY_H

#include <boost/math/interpolators/cardinal_cubic_b_spline.hpp>
#include <filesystem>
#include <vector>

namespace pyvrp::velocity
{
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
    WLTCProfile(const std::string name,
                const std::filesystem::path path,
                const int startOffsetTime,
                const int endOffsetTime);
    double getDistanceForTravelTime(double time);
    double getSquaredVelocityIntegral(double time);
    double getCubedVelocityIntegral(double time);
    double getTimeForTravelDistance(double distance);

    std::string name() const { return name_; }
    std::string path() const { return path_; }
    std::vector<double> time() { return time_; }
    std::vector<double> velocity() { return velocity_; }
    boost::math::interpolators::cardinal_cubic_b_spline<double> fullSpline()
    {
        return fullSpline_;
    }
    double fullProfileDistance() { return fullProfileDistance_; }
    double repeatableProfileDistance() { return repeatableProfileDistance_; }
    double repeatableProfileTime() { return repeatableProfileTime_; }
    double startOffsetTime() { return startOffsetTime_; }
    double endOffsetTime() { return endOffsetTime_; }
    double startOffsetDistance() { return startOffsetDistance_; }
    double endOffsetDistance() { return endOffsetDistance_; }
    std::vector<double> splineTime()
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
    std::vector<double> splineVelocity()
    {
        return std::vector<double>(velocity_.begin() + startOffsetTime_,
                                   velocity_.end() - endOffsetTime_);
    }
};

// WLTCProfile slowVelocityProfile, mediumVelocityProfile, highVelocityProfile;

WLTCProfile getProfileBasedOnDistance(double distance);
}  // namespace pyvrp::velocity

#endif  // PYVRP_VELOCITY_H
