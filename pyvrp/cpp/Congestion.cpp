#include "Congestion.h"
#include "includes/csv.hpp"
#include <boost/math/interpolators/cardinal_cubic_b_spline.hpp>
#include <boost/math/quadrature/trapezoidal.hpp>

namespace pyvrp::congestion
{
CongestionProfile::CongestionProfile(const std::filesystem::path &path)
    : path_(path)
{
    csv::CSVReader congestionCsvFile(std::filesystem::absolute(path_).string());
    for (csv::CSVRow row : congestionCsvFile)
    {
        // Congestion profiles are expected to have two columns: time and
        // congestion. Time in seconds and congestion in percentage.
        double t = row["time"].get<double>() * 3600;  // Convert to seconds
        time_.push_back(t);
        double c = row["congestion"].get<double>();  // Convert to fraction
        congestion_.push_back(c);
    }

    double stepSize = (time_.back() - time_.front()) / (time_.size() - 1);
    spline_ = boost::math::interpolators::cardinal_cubic_b_spline<double>(
        congestion_.begin(), congestion_.end(), time_.front(), stepSize);

    std::transform(squaredCongestion_.begin(),
                   squaredCongestion_.end(),
                   std::back_inserter(squaredCongestion_),
                   [](double x) { return x * x; });
    squaredSpline_ = boost::math::interpolators::cardinal_cubic_b_spline(
        squaredCongestion_.begin(),
        squaredCongestion_.end(),
        squaredCongestion_.front(),
        stepSize);

    std::transform(cubedCongestion_.begin(),
                   cubedCongestion_.end(),
                   std::back_inserter(cubedCongestion_),
                   [](double x) { return x * x * x; });
    cubedSpline_ = boost::math::interpolators::cardinal_cubic_b_spline(
        cubedCongestion_.begin(),
        cubedCongestion_.end(),
        cubedCongestion_.front(),
        stepSize);
}

double CongestionProfile::getCongestionIntegral(double const &from,
                                                double const &to) const
{
    return boost::math::quadrature::trapezoidal(spline_, from, to);
}

double CongestionProfile::getCongestionIntegral(Duration const &from,
                                                Duration const &to) const
{
    return getCongestionIntegral(static_cast<double>(from),
                                 static_cast<double>(to));
}

double CongestionProfile::getSquaredCongestionIntegral(double const &from,
                                                       double const &to) const
{
    return boost::math::quadrature::trapezoidal(squaredSpline_, from, to);
}

double CongestionProfile::getSquaredCongestionIntegral(Duration const &from,
                                                       Duration const &to) const
{
    return getSquaredCongestionIntegral(static_cast<double>(from),
                                        static_cast<double>(to));
}

double CongestionProfile::getCubedCongestionIntegral(double const &from,
                                                     double const &to) const
{
    return boost::math::quadrature::trapezoidal(cubedSpline_, from, to);
}

double CongestionProfile::getCubedCongestionIntegral(Duration const &from,
                                                     Duration const &to) const
{
    return getCubedCongestionIntegral(static_cast<double>(from),
                                      static_cast<double>(to));
}

CongestionProfile const getCongestionProfile(
    [[maybe_unused]] CongestionBehaviour const congestionBehaviour)
{
    std::filesystem::path currentFile = __FILE__;
    // We run this in the pyvrp folder (you can see
    // the .so file)
    std::filesystem::path rootDir = currentFile.parent_path().parent_path();
    std::filesystem::path congestionCsv
        = rootDir / "research" / "data" / "congestion" / "congestion.csv";
    return CongestionProfile(congestionCsv);
}
}  // namespace pyvrp::congestion