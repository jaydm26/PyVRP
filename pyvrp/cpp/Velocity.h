#ifndef PYVRP_VELOCITY_H
#define PYVRP_VELOCITY_H

#include "includes/csv.hpp"
#include <boost/math/interpolators/cubic_b_spline.hpp>
#include <boost/math/quadrature/trapezoidal.hpp>
#include <vector>

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

void initialize();

double get_velocity_integral(double start, double end, int power = 1);
}  // namespace velocity

#endif  // PYVRP_VELOCITY_H
