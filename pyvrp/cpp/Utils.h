#include <cassert>
#ifndef PYVRP_UTILS_H
#define PYVRP_UTILS_H

namespace pyvrp::utils
{
// Do not attempt to modify velocity here. Consider this as a utility function
// Power To Mass Ratio is in W per kg (which is numerically equal to KW per
// ton), velocity is in m/s, and the result is factor per g of CO2 per ton per
// hour.
/**
 * Get the emission cost per ton per hour when the velocity is constant. For
 * cases where one must provide congestion, provide the congested velocity
 * instead of the uncongested velocity.
 */
inline double
emissionCostPerTonPerHourConstantVelocity(double const powerToMassRatio,
                                          double const velocity)
{
    double internaVelocity = velocity * 3.6;  // convert to km/hr
    double a, b, c, d;
    a = 465.390 + 48.143 * powerToMassRatio;
    b = (32.389 + 0.8931 * powerToMassRatio) * internaVelocity;
    c = (-0.4771 - 0.02559 * powerToMassRatio) * internaVelocity
        * internaVelocity;
    d = (0.0008889 + 0.0004055 * powerToMassRatio) * internaVelocity
        * internaVelocity * internaVelocity;

    return (a + b + c + d) / 1000.0;  // Convert to kg
}

/**
 * Emission cost per ton per hour when the velocity is non-linear. Duration is
 * obtained by solving:
 *
 * integral_ti^tj [v_ij(t)] dt = d_ij.
 *
 * In the above, the we know t_i, v_ij(t) and d_ij. Thus, we can solve for t_j
 * which then provides duration. Squared velocity integral and cubed velocity
 * integral are obtained directly from the WLTCProfile.
 */
inline double
emissionFactorPerTonNonLinearVelocity(double const powerToMassRatio,
                                      double const congestion,
                                      double const duration,  // input in s
                                      double const distance,  // input in m
                                      double const squaredVelocityIntegral,
                                      double const cubedVelocityIntegral)
{
    double a, b, c, d;
    a = (465.390 + 48.143 * powerToMassRatio) * duration
        / 3600.0;  // convert to hours
    b = (32.389 + 0.8931 * powerToMassRatio) * distance * congestion
        / 1000.0;  // convert to km
    c = (-0.4771 - 0.02559 * powerToMassRatio) * squaredVelocityIntegral
        * congestion / 1000 / 1000 * 3600;  // convert to km/hr
    d = (0.0008889 + 0.0004055 * powerToMassRatio) * cubedVelocityIntegral
        * congestion / 1000 / 1000 / 1000 * 3600 * 3600;  // Convert to km/hr

    assert(a + b + c + d >= 0);       // Ensure the result is non-negative
    return (a + b + c + d) / 1000.0;  // Convert to kg
}
}  // namespace pyvrp::utils

#endif  // PYVRP_UTILS_H