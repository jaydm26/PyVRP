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
 *
 * The result of the function is in kg of CO2 per ton per hour. To obtain the kg
 * of CO2 per ton, multiply by the congested duration in hours. Since the
 * congestion and velocity for these calculations is constant (in segments or
 * throughout), congested duration would be obtained by dividing the uncongested
 * duration by congestion.
 */
inline double
emissionCostPerTonPerHourConstantVelocity(double const powerToMassRatio,
                                          double const velocity)
{
    double internalVelocity = velocity * 3.6;  // convert to km/hr
    double a, b, c, d;
    a = 465.390 + 48.143 * powerToMassRatio;
    b = (32.389 + 0.8931 * powerToMassRatio) * internalVelocity;
    c = (-0.4771 - 0.02559 * powerToMassRatio) * internalVelocity
        * internalVelocity;
    d = (0.0008889 + 0.0004055 * powerToMassRatio) * internalVelocity
        * internalVelocity * internalVelocity;

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
 *
 * In this, we expect congestion to be constant (either in segment or
 * throughout). Duration is always in the congested form. Distance and the
 * subsequent high power integrals of velocity are also obtained in the
 * congested form.
 */
inline double emissionFactorPerTonNonLinearVelocity(
    double const powerToMassRatio,
    double const constantTerm,  // if velocity, provide in m/s, if congestion,
                                // provide in fraction
    double const congestedDuration,  // input in s
    double const linearIntegral,     // input in m
    double const squaredIntegral,
    double const cubedIntegral)
{
    double a, b, c, d;
    a = (465.390 + 48.143 * powerToMassRatio) * congestedDuration
        / 3600.0;  // convert to hours
    b = (32.389 + 0.8931 * powerToMassRatio) * linearIntegral * constantTerm
        / 1000.0;  // convert to km
    c = (-0.4771 - 0.02559 * powerToMassRatio) * squaredIntegral * constantTerm
        * constantTerm / 1000 / 1000 * 3600;  // convert to km/hr
    d = (0.0008889 + 0.0004055 * powerToMassRatio) * cubedIntegral
        * constantTerm * constantTerm * constantTerm / 1000 / 1000 / 1000 * 3600
        * 3600;  // Convert to km/hr

    assert(a + b + c + d >= 0);       // Ensure the result is non-negative 
    return (a + b + c + d) / 1000.0;  // Convert to kg
}

inline double emissionFactorPerTonNonLinearVelocityAndCongestion(
    double const powerToMassRatio,
    double const congestedDuration,  // input in s
    double const linearIntegral,     // input in m
    double const squaredIntegral,
    double const cubedIntegral)
{
    double a, b, c, d;
    a = (465.390 + 48.143 * powerToMassRatio) * congestedDuration
        / 3600.0;  // convert to hours
    b = (32.389 + 0.8931 * powerToMassRatio) * linearIntegral
        / 1000.0;  // convert to km
    c = (-0.4771 - 0.02559 * powerToMassRatio) * squaredIntegral / 1000 / 1000
        * 3600;  // convert to km/hr
    d = (0.0008889 + 0.0004055 * powerToMassRatio) * cubedIntegral / 1000 / 1000
        / 1000 * 3600 * 3600;  // Convert to km/hr

    assert(a + b + c + d >= 0);       // Ensure the result is non-negative
    return (a + b + c + d) / 1000.0;  // Convert to kg
}
}  // namespace pyvrp::utils

#endif  // PYVRP_UTILS_H