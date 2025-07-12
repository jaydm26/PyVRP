#include "Congestion.h"
#include "Velocity.h"

namespace pyvrp::congestedVelocity
{

class CongestedWLTCProfile
{
    pyvrp::velocity::WLTCProfile wltcProfile_;
    pyvrp::congestion::CongestionProfile congestionProfile_;

public:
    CongestedWLTCProfile(
        pyvrp::velocity::WLTCProfile const &wltcProfile,
        pyvrp::congestion::CongestionProfile const &congestionProfile)
        : wltcProfile_(wltcProfile), congestionProfile_(congestionProfile)
    {
    }
    double getDistanceForCongestedTravelTime(double const &from,
                                             double const &to) const;
    double
    getSquaredVelocityIntegralForCongestedTravelTime(double const &from,
                                                     double const &to) const;
    double
    getCubedVelocityIntegralForCongestedTravelTime(double const &from,
                                                   double const &to) const;
    double getCongestedTravelTimeForDistance(double const &distance,
                                             double const &from) const;
};

}  // namespace pyvrp::congestedVelocity