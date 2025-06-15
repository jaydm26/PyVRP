#include "Velocity.h"
#include <iostream>

namespace velocity
{
inline std::vector<double> time;
inline std::vector<double> velocity_1, velocity_2, velocity_3;
inline boost::math::cubic_b_spline<double> spline_1, spline_2, spline_3;
/**
 * Initializes the velocity data from a CSV file.
 * The CSV file should have columns "time" and "velocity".
 * The velocity is processed to create three splines:
 * - spline_1 for velocity
 * - spline_2 for velocity squared
 * - spline_3 for velocity cubed
 */

void initialize()
{
    csv::CSVReader reader("data.csv");
    for (csv::CSVRow row : reader)
    {
        time.push_back(row["time"].get<double>());
        double v = row["velocity"].get<double>();
        velocity_1.push_back(v);
        velocity_2.push_back(v * v);
        velocity_3.push_back(v * v * v);
    }
    if (!velocity_1.empty())
    {
        spline_1 = boost::math::cubic_b_spline<double>(
            velocity_1.begin(),
            velocity_1.end(),
            time.front(),
            (time.back() - time.front()) / (time.size() - 1));
        spline_2 = boost::math::cubic_b_spline<double>(
            velocity_2.begin(),
            velocity_2.end(),
            time.front(),
            (time.back() - time.front()) / (time.size() - 1));
        spline_3 = boost::math::cubic_b_spline<double>(
            velocity_3.begin(),
            velocity_3.end(),
            time.front(),
            (time.back() - time.front()) / (time.size() - 1));
    }
}

double get_velocity_integral(double start, double end, int power = 1)
{
    inline auto function_1 = [](double x) { return spline_1(x); };
    inline auto function_2 = [](double x) { return spline_2(x); };
    inline auto function_3 = [](double x) { return spline_3(x); };
    switch (power)
    {
    case 1:
        return boost::math::quadrature::trapezoidal<double>(
            function_1, start, end);
        break;

    case 2:
        return boost::math::quadrature::trapezoidal<double>(
            function_2, start, end);
        break;

    case 3:
        return boost::math::quadrature::trapezoidal<double>(
            function_3, start, end);
        break;

    default:
        return 0.0;
    }
}
}  // namespace velocity
