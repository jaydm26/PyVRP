#include "Velocity.h"
#include "includes/csv.hpp"
#include <boost/math/interpolators/cardinal_cubic_b_spline.hpp>
#include <boost/math/quadrature/trapezoidal.hpp>
#include <boost/math/tools/roots.hpp>
#include <filesystem>
#include <vector>

namespace pyvrp::velocity
{

WLTCProfile::WLTCProfile(const std::string name,
                         const std::filesystem::path path,
                         const int startOffsetTime,
                         const int endOffsetTime)
    : name_(name),
      path_(path),
      startOffsetTime_(startOffsetTime),
      endOffsetTime_(endOffsetTime)
{
    csv::CSVReader fullCsvFile(std::filesystem::absolute(path_).string());
    for (csv::CSVRow row : fullCsvFile)
    {
        // WLTC profiles are expected to have two columns: time and velocity.
        // WLTC profiles time in seconds and velocity in km/hr.
        double t = row["time"].get<double>();
        time_.push_back(t);
        double v = row["velocity"].get<double>() * 1000.0
                   / 3600.0;  // Convert to m/s
        velocity_.push_back(v);
    }

    double stepSize = (time_.back() - time_.front()) / (time_.size() - 1);

    fullSpline_ = boost::math::interpolators::cardinal_cubic_b_spline<double>(
        velocity_.begin(), velocity_.end(), time_.front(), stepSize);
    fullProfileDistance_ = boost::math::quadrature::trapezoidal(
        fullSpline_, time_.front(), time_.back());

    // Filter for the profile and create the repeatable spline data
    auto splineTime_ = splineTime();
    auto splineVelocity_ = splineVelocity();

    spline_ = boost::math::interpolators::cardinal_cubic_b_spline<double>(
        splineVelocity_.begin(),
        splineVelocity_.end(),
        splineTime_.front(),
        stepSize);

    repeatableProfileDistance_
        = boost::math::quadrature::trapezoidal(
              spline_, splineTime_.front(), splineTime_.back())
          / 3600;
    repeatableProfileTime_ = splineTime_.back() - splineTime_.front();

    if (time_.front() < time_[startOffsetTime_])
    {
        startOffsetDistance_
            = boost::math::quadrature::trapezoidal(
                  fullSpline_, time_.front(), time_[startOffsetTime_])
              / 3600;
    }
    else
    {
        startOffsetDistance_ = 0.0;
    }

    if (time_[time_.size() - 1 - endOffsetTime_] < time_.back())
    {
        endOffsetDistance_ = boost::math::quadrature::trapezoidal(
                                 fullSpline_,
                                 time_[time_.size() - 1 - endOffsetTime_],
                                 time_.back())
                             / 3600;
    }
    else
    {
        endOffsetDistance_ = 0.0;
    }

    std::transform(splineVelocity_.begin(),
                   splineVelocity_.end(),
                   std::back_inserter(squaredVelocity_),
                   [](double x) { return x * x; });
    squaredSpline_ = boost::math::interpolators::cardinal_cubic_b_spline(
        squaredVelocity_.begin(),
        squaredVelocity_.end(),
        splineTime_.front(),
        stepSize);

    repeatableSquaredVelocityIntegral_
        = boost::math::quadrature::trapezoidal(
              squaredSpline_, splineTime_.front(), splineTime_.back())
          / 3600;

    std::transform(splineVelocity_.begin(),
                   splineVelocity_.end(),
                   std::back_inserter(cubedVelocity_),
                   [](double x) { return x * x * x; });
    cubedSpline_ = boost::math::interpolators::cardinal_cubic_b_spline(
        cubedVelocity_.begin(),
        cubedVelocity_.end(),
        splineTime_.front(),
        stepSize);

    repeatableCubedVelocityIntegral_
        = boost::math::quadrature::trapezoidal(
              cubedSpline_, splineTime_.front(), splineTime_.back())
          / 3600;
}

/**
 * Distance is obtained in km. To adjust the velocity in km/hr and time in s, we
 * divide the final value by 3600. Since we want to obfuscate the complexity of
 * the setup, we accept the time in seconds.
 */
double WLTCProfile::getDistanceForTravelTime(double time)
{
    double distance = std::floor(time / repeatableProfileTime_)
                      * repeatableProfileDistance_;
    double remainingTime = std::fmod(time, repeatableProfileTime_);
    double remainingDistance
        = boost::math::quadrature::trapezoidal(spline_, 0.0, remainingTime)
          / 3600;
    distance += remainingDistance;

    return distance;
}

double WLTCProfile::getSquaredVelocityIntegral(double time)
{
    double value = std::floor(time / repeatableProfileTime_)
                   * repeatableSquaredVelocityIntegral_;
    double remainingTime = std::fmod(time, repeatableProfileTime_);
    double remainingSquaredVelocityIntegral
        = boost::math::quadrature::trapezoidal(
              squaredSpline_, 0.0, remainingTime)
          / 3600;
    value += remainingSquaredVelocityIntegral;
    return value;
}

double WLTCProfile::getCubedVelocityIntegral(double time)
{
    double value = std::floor(time / repeatableProfileTime_)
                   * repeatableCubedVelocityIntegral_;
    double remainingTime = std::fmod(time, repeatableProfileTime_);
    double remainingCubedVelocityIntegral
        = boost::math::quadrature::trapezoidal(cubedSpline_, 0.0, remainingTime)
          / 3600;
    value += remainingCubedVelocityIntegral;
    return value;
}

/**
 * Distance must be passed in km. The value returned will be time in seconds.
 */
double WLTCProfile::getTimeForTravelDistance(double distance)
{
    // We assume that distance will always be greater than the
    // fullProfileDistance_ for the high velocity scenario. Otherwise it
    // becomes highly complex.

    // We start with the offset distances already.
    double remainingDistance
        = distance - startOffsetDistance_ - endOffsetDistance_;
    // This means that the vehicle must have travelled at least the offset
    // times.
    double time = startOffsetTime_ + endOffsetTime_;
    // The remainder of the distance can be obtained by first counting
    // number of full repeatable splines we can go through. The remaining
    // distance thereafter can be passed through a bisection algorithm to
    // obtain the amount of time it will take to complete that remaining
    // distance

    // Cover the repeatable profiles
    int repeatableProfiles
        = std::floor(remainingDistance / repeatableProfileDistance_);
    remainingDistance -= repeatableProfiles * repeatableProfileDistance_;
    time += repeatableProfiles * repeatableProfileTime_;

    // The remainder distance must be covered in time t which is obtained by
    // finding the root of the function below (i.e. when it is equal to 0).
    auto func = [this, remainingDistance](double t)
    {
        return boost::math::quadrature::trapezoidal(spline_, 0.0, t) / 3600
               - remainingDistance;
    };
    // To get the root, we use bisection.
    auto result = boost::math::tools::bisect(
        func,
        time_.front(),
        time_.back(),
        [](double a, double b) { return std::abs(a - b) <= 1e-8; });
    // Take the average value of the two results we get and add that to the
    // time. This is the total time it takes to cover the distance.
    time += (result.first + result.second) / 2;
    return time;
}

std::filesystem::path currentFile = __FILE__;
// We run this in the pyvrp folder (you can see
// the .so file)
std::filesystem::path rootDir = currentFile.parent_path().parent_path();

std::filesystem::path pathToCsv = rootDir / "research" / "data" / "wltc";
std::filesystem::path slowVelocityProfilePath = pathToCsv / "SlowProfile.csv";
std::filesystem::path mediumVelocityProfilePath
    = pathToCsv / "MediumProfile.csv";
std::filesystem::path highVelocityProfilePath = pathToCsv / "HighProfile.csv";

WLTCProfile slowVelocityProfile
    = WLTCProfile("slow", slowVelocityProfilePath, 0, 0);
WLTCProfile mediumVelocityProfile
    = WLTCProfile("medium", mediumVelocityProfilePath, 10, 36);
WLTCProfile highVelocityProfile
    = WLTCProfile("high", highVelocityProfilePath, 70, 113);

WLTCProfile getProfileBasedOnDistance(double distance)
{
    if (distance > 3 * mediumVelocityProfile.fullProfileDistance())
    {
        return highVelocityProfile;
    }
    else if (distance > 3 * slowVelocityProfile.fullProfileDistance())
    {
        return mediumVelocityProfile;
    }
    else
    {
        return slowVelocityProfile;
    }
}

}  // namespace pyvrp::velocity