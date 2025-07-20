#include "Congestion.h"
#include "includes/csv.hpp"

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

    assert(time_.size() > 0);
    double stepSize = (time_.back() - time_.front()) / (time_.size() - 1);
    spline_ = boost::math::interpolators::cardinal_cubic_b_spline<double>(
        congestion_.begin(), congestion_.end(), time_.front(), stepSize);

    std::transform(congestion_.begin(),
                   congestion_.end(),
                   std::back_inserter(squaredCongestion_),
                   [](double x) { return x * x; });
    squaredSpline_ = boost::math::interpolators::cardinal_cubic_b_spline(
        squaredCongestion_.begin(),
        squaredCongestion_.end(),
        time_.front(),
        stepSize);

    std::transform(congestion_.begin(),
                   congestion_.end(),
                   std::back_inserter(cubedCongestion_),
                   [](double x) { return x * x * x; });
    cubedSpline_ = boost::math::interpolators::cardinal_cubic_b_spline(
        cubedCongestion_.begin(),
        cubedCongestion_.end(),
        time_.front(),
        stepSize);

    std::transform(congestion_.begin(),
                   congestion_.end(),
                   std::back_inserter(quadrupledCongestion_),
                   [](double x) { return x * x * x * x; });
    quadrupledSpline_ = boost::math::interpolators::cardinal_cubic_b_spline(
        quadrupledCongestion_.begin(),
        quadrupledCongestion_.end(),
        time_.front(),
        stepSize);

    std::transform(congestion_.begin(),
                   congestion_.end(),
                   std::back_inserter(pentupledCongestion_),
                   [](double x) { return x * x * x * x * x; });
    pentupledSpline_ = boost::math::interpolators::cardinal_cubic_b_spline(
        pentupledCongestion_.begin(),
        pentupledCongestion_.end(),
        time_.front(),
        stepSize);

    std::transform(congestion_.begin(),
                   congestion_.end(),
                   std::back_inserter(hextupledCongestion_),
                   [](double x) { return x * x * x * x * x * x; });
    hextupledSpline_ = boost::math::interpolators::cardinal_cubic_b_spline(
        hextupledCongestion_.begin(),
        hextupledCongestion_.end(),
        time_.front(),
        stepSize);
}

double CongestionProfile::getCongestionIntegral(double const &from,
                                                double const &to) const
{
    // Integrate 1 - 0.5 * gamma - 0.3 * gamma^2 where gamma is the value of the
    // spline at time t.
    auto const linearIntegral
        = boost::math::quadrature::trapezoidal(spline_, from, to);
    auto const squaredIntegral
        = boost::math::quadrature::trapezoidal(squaredSpline_, from, to);

    auto const a = to - from;
    auto const b = 0.5 * linearIntegral;
    auto const c = 0.3 * squaredIntegral;
    return a - b - c;
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
    // Integrate (1 - 0.5 * gamma - 0.3 * gamma^2)^2 where gamma is the value of
    // the spline at time t.
    auto const linearIntegral
        = boost::math::quadrature::trapezoidal(spline_, from, to);
    auto const squaredIntegral
        = boost::math::quadrature::trapezoidal(squaredSpline_, from, to);
    auto const cubedIntegral
        = boost::math::quadrature::trapezoidal(cubedSpline_, from, to);
    auto const quadrupledIntegral
        = boost::math::quadrature::trapezoidal(quadrupledSpline_, from, to);
    auto const a = to - from;
    auto const b = -1 * linearIntegral;
    auto const c = -0.35 * squaredIntegral;
    auto const d = 0.3 * cubedIntegral;
    auto const e = 0.09 * quadrupledIntegral;
    return a + b + c + d + e;
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
    // Integrate (1 - 0.5 * gamma - 0.3 * gamma^2)^3 where gamma is the value of
    // the spline at time t.
    auto const linearIntegral
        = boost::math::quadrature::trapezoidal(spline_, from, to);
    auto const squaredIntegral
        = boost::math::quadrature::trapezoidal(squaredSpline_, from, to);
    auto const cubedIntegral
        = boost::math::quadrature::trapezoidal(cubedSpline_, from, to);
    auto const quadrupledIntegral
        = boost::math::quadrature::trapezoidal(quadrupledSpline_, from, to);
    auto const pentupledIntegral
        = boost::math::quadrature::trapezoidal(pentupledSpline_, from, to);
    auto const hextupledIntegral
        = boost::math::quadrature::trapezoidal(hextupledSpline_, from, to);
    auto const a = to - from;
    auto const b = -1.5 * linearIntegral;
    auto const c = -0.15 * squaredIntegral;
    auto const d = 0.775 * cubedIntegral;
    auto const e = 0.045 * quadrupledIntegral;
    auto const f = -0.135 * pentupledIntegral;
    auto const g = -0.027 * hextupledIntegral;
    return a + b + c + d + e + f + g;
}

double CongestionProfile::getCubedCongestionIntegral(Duration const &from,
                                                     Duration const &to) const
{
    return getCubedCongestionIntegral(static_cast<double>(from),
                                      static_cast<double>(to));
}

double CongestionProfile::getDurationBasedOnDistanceAndVelocity(
    double const &distance, double const &velocity, double const &now) const
{
    auto const target = distance / velocity;
    auto func = [this, now, target](double t)
    {
        double const linearIntegral
            = boost::math::quadrature::trapezoidal(spline_, now, t);
        double const squaredIntegral
            = boost::math::quadrature::trapezoidal(squaredSpline_, now, t);
        auto const a = now - t;
        auto const b = 0.5 * linearIntegral;
        auto const c = 0.3 * squaredIntegral;
        return a - b - c - target;
    };
    // To get the root, we use bisection.
    auto result = boost::math::tools::bisect(
        func,
        now,
        time_.back(),
        [](double a, double b) { return std::abs(a - b) <= 1e-8; });
    return (result.first + result.second) / 2;  // in seconds
}

CongestionProfile const getCongestionProfile(
    [[maybe_unused]] CongestionBehaviour const congestionBehaviour)
{
    std::filesystem::path currentFile = __FILE__;
    // We run this in the pyvrp folder (you can see
    // the .so file)
    std::filesystem::path rootDir = currentFile.parent_path().parent_path().parent_path();
    std::filesystem::path congestionCsv;
    if (congestionBehaviour == CongestionBehaviour::ConstantCongestion)
    {
        // For constant congestion, we use the default congestion profile.
        congestionCsv = rootDir / "PyVRP" / "research" / "data" / "congestion"
                        / "constant_congestion.csv";
    }
    else
    {
        congestionCsv
            = rootDir / "PyVRP" / "research" / "data" / "congestion" / "congestion.csv";
    }
    return CongestionProfile(congestionCsv);
}
}  // namespace pyvrp::congestion