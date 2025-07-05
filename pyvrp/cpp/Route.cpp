#include "Route.h"
#include "Congestion.h"
#include "DurationSegment.h"
#include "LoadSegment.h"
#include "Utils.h"
#include "Velocity.h"

#include <algorithm>
#include <cassert>
#include <fstream>
#include <numeric>

using pyvrp::Cost;
using pyvrp::Distance;
using pyvrp::Duration;
using pyvrp::Load;
using pyvrp::Route;
using pyvrp::Trip;

using Client = size_t;

Route::Iterator::Iterator(Route const &route, size_t idx)
    : route_(&route), trip_(route.numTrips()), idx_(0)
{
    assert(idx <= route.size());

    auto const &trips = route.trips();
    for (size_t trip = 0; trip != trips.size(); ++trip)
    {
        if (idx < trips[trip].size())
        {
            trip_ = trip;
            idx_ = idx;
            break;
        }

        idx -= trips[trip].size();
    }
}

bool Route::Iterator::operator==(Iterator const &other) const
{
    return route_ == other.route_ && trip_ == other.trip_ && idx_ == other.idx_;
}

Client Route::Iterator::operator*() const
{
    auto const &trips = route_->trips();
    assert(trip_ < trips.size());
    assert(idx_ < trips[trip_].size());

    return trips[trip_][idx_];
}

Route::Iterator Route::Iterator::operator++(int)
{
    auto tmp = *this;
    ++*this;
    return tmp;
}

Route::Iterator &Route::Iterator::operator++()
{
    auto const &trips = route_->trips();
    if (idx_ + 1 < trips[trip_].size())
    {
        ++idx_;
        return *this;
    }

    // Then we move to the next trip. This trip could be empty - in that case
    // we continue to the next until we either exhaust all trips, or we find a
    // non-empty trip.
    ++trip_;
    while (trip_ < trips.size() && trips[trip_].empty())
        ++trip_;

    idx_ = 0;
    return *this;
}

Route::ScheduledVisit::ScheduledVisit(size_t location,
                                      size_t trip,
                                      Duration startService,
                                      Duration endService,
                                      Duration waitDuration,
                                      Duration timeWarp)
    : location(location),
      trip(trip),
      startService(startService),
      endService(endService),
      waitDuration(waitDuration),
      timeWarp(timeWarp)
{
    assert(startService <= endService);
}

Duration Route::ScheduledVisit::serviceDuration() const
{
    return endService - startService;
}

void Route::validate(ProblemData const &data) const
{
    auto const &vehData = data.vehicleType(vehicleType_);

    if (trips_.size() > vehData.maxTrips())
        throw std::invalid_argument("Vehicle cannot perform this many trips.");

    if (trips_[0].startDepot() != startDepot_)
    {
        auto const *msg = "Route must start at vehicle's start_depot.";
        throw std::invalid_argument(msg);
    }

    if (trips_.back().endDepot() != endDepot_)
        throw std::invalid_argument("Route must end at vehicle's end_depot.");

    for (auto const &trip : trips_)
        if (trip.vehicleType() != vehicleType_)
        {
            auto const *msg = "Each trip must use the route's vehicle type.";
            throw std::invalid_argument(msg);
        }

    for (size_t idx = 0; idx + 1 != trips_.size(); ++idx)
        if (trips_[idx].endDepot() != trips_[idx + 1].startDepot())
        {
            auto *msg = "Consecutive trips must start at previous' end_depot.";
            throw std::invalid_argument(msg);
        }
}

void Route::makeSchedule(ProblemData const &data)
{
    schedule_.clear();
    schedule_.reserve(size() + numTrips() + 1);  // clients and depots

    auto const &vehData = data.vehicleType(vehicleType_);
    auto const &durations = data.durationMatrix(vehData.profile);
    auto const congestionProfile
        = pyvrp::congestion::getCongestionProfile(data.congestionBehaviour());

    auto now = startTime_;
    auto const handle
        = [&](auto const &where, size_t location, size_t trip, Duration service)
    {
        auto const wait = std::max<Duration>(where.twEarly - now, 0);
        auto const tw = std::max<Duration>(now - where.twLate, 0);

        now += wait;
        now -= tw;

        schedule_.emplace_back(location, trip, now, now + service, wait, tw);

        now += service;
    };

    for (size_t tripIdx = 0; tripIdx != trips_.size(); ++tripIdx)
    {
        auto const &trip = trips_[tripIdx];
        ProblemData::Depot const &start = data.location(trip.startDepot());

        auto const earliestStart = std::max(
            start.twEarly, std::min(trip.releaseTime(), start.twLate));
        auto const wait = std::max<Duration>(earliestStart - now, 0);
        auto const tw = std::max<Duration>(now - start.twLate, 0);

        now += wait;
        now -= tw;

        schedule_.emplace_back(trip.startDepot(), tripIdx, now, now, wait, tw);

        size_t prevClient = trip.startDepot();
        for (auto const client : trip)
        {
            auto const congestion
                = data.congestionBehaviour()
                          == pyvrp::congestion::CongestionBehaviour::
                              ConstantCongestion
                      ? vehData.congestion
                      : congestionProfile.getCongestionValue(now);
            auto const edgeDur = durations(prevClient, client).get()
                                 / congestion;  // Insert congestion here
            now += edgeDur;

            ProblemData::Client const &clientData = data.location(client);
            handle(clientData, client, tripIdx, clientData.serviceDuration);

            prevClient = client;
        }
        auto const congestion = congestionProfile.getCongestionValue(now);
        auto const edgeDur
            = durations(prevClient, trip.endDepot()).get() / congestion;
        now += edgeDur;
    }

    ProblemData::Depot const &end = data.location(endDepot_);
    handle(end, endDepot_, numTrips(), 0);
}

Route::Route(ProblemData const &data, Visits visits, size_t vehicleType)
    : Route(data, {{data, std::move(visits), vehicleType}}, vehicleType)
{
}

Route::Route(ProblemData const &data, Trips trips, size_t vehType)
    : trips_(std::move(trips)),
      delivery_(data.numLoadDimensions(), 0),
      pickup_(data.numLoadDimensions(), 0),
      excessLoad_(data.numLoadDimensions(), 0),
      vehicleType_(vehType)
{
    if (trips_.empty())  // then we insert a dummy trip for ease.
        trips_.emplace_back(data, Visits{}, vehType);

    auto const &vehData = data.vehicleType(vehType);
    startDepot_ = vehData.startDepot;
    endDepot_ = vehData.endDepot;

    validate(data);
    travel_ = 0;

    for (auto const &trip : trips_)  // general statistics
    {
        distance_ += trip.distance();
        service_ += trip.serviceDuration();
        travel_ += trip.travelDuration(travel_.get(), data);
        prizes_ += trip.prizes();

        auto const [x, y] = trip.centroid();
        centroid_.first += (x * trip.size()) / size();
        centroid_.second += (y * trip.size()) / size();
    }

    distanceCost_ = vehData.unitDistanceCost * static_cast<Cost>(distance_);
    excessDistance_ = std::max<Distance>(distance_ - vehData.maxDistance, 0);

    for (size_t idx = 0; idx != trips_.size(); ++idx)  // load statistics
    {
        auto const &trip = trips_[idx];
        auto const &tripDeliv = trip.delivery();
        auto const &tripPick = trip.pickup();
        auto const &tripLoad = trip.load();

        for (size_t dim = 0; dim != data.numLoadDimensions(); ++dim)
        {
            LoadSegment ls = {tripDeliv[dim], tripPick[dim], tripLoad[dim], 0};

            if (idx == 0 && vehData.initialLoad[dim] > 0)
                // This is initial load that the first trip does not know about
                // that we need to account for first.
                ls = LoadSegment::merge({vehData, dim}, ls);

            delivery_[dim] += ls.delivery();
            pickup_[dim] += ls.pickup();
            excessLoad_[dim] += ls.excessLoad(vehData.capacity[dim]);
        }
    }

    // OLD: Duration statistics. We iterate in reverse, that is, from the last
    // to the first visit.

    // UPDATED: We iterate in the normal direction as we will need the time just
    // before leaving each node to obtain congestion.
    auto const &durations = data.durationMatrix(vehData.profile);
    // DurationSegment ds = {vehData, vehData.twLate};
    // for (auto trip = trips_.rbegin(); trip != trips_.rend(); ++trip)
    // {
    //     ProblemData::Depot const &end = data.location(trip->endDepot());
    //     ds = DurationSegment::merge(0, {end}, ds);

    //     size_t nextClient = trip->endDepot();
    //     for (auto it = trip->rbegin(); it != trip->rend(); ++it)
    //     {
    //         auto const client = *it;
    //         auto const edgeDuration = durations(client, nextClient);
    //         ProblemData::Client const &clientData = data.location(client);

    //         ds = DurationSegment::merge(edgeDuration, {clientData}, ds);
    //         nextClient = client;
    //     }

    //     auto const edgeDuration = durations(trip->startDepot(), nextClient);
    //     ProblemData::Depot const &start = data.location(trip->startDepot());
    //     DurationSegment const depotDS = {start};

    //     ds = DurationSegment::merge(edgeDuration, depotDS, ds);
    //     ds = ds.finaliseFront();
    // }

    // ds = DurationSegment::merge(0, {vehData, vehData.startLate}, ds);

    auto const congestionProfile
        = pyvrp::congestion::getCongestionProfile(data.congestionBehaviour());
    DurationSegment ds = {vehData, vehData.startLate};
    for (auto trip = trips_.begin(); trip != trips_.end(); ++trip)
    {
        ProblemData::Depot const &start = data.location(trip->startDepot());
        ds = DurationSegment::merge(0, ds, {start});

        auto tripReleaseTime = trip->releaseTime();
        size_t prevClient = trip->startDepot();
        for (auto it = trip->begin(); it != trip->end(); ++it)
        {
            auto const client = *it;
            double congestion = congestionProfile.getCongestionValue(
                tripReleaseTime + ds.duration());
            auto const edgeDuration = static_cast<Duration>(
                durations(prevClient, client).get() / congestion);
            ProblemData::Client const &clientData = data.location(client);

            ds = DurationSegment::merge(edgeDuration, ds, {clientData});
            prevClient = client;
        }

        double congestion = congestionProfile.getCongestionValue(
            tripReleaseTime + ds.duration());
        auto const edgeDuration = static_cast<Duration>(
            durations(prevClient, trip->endDepot()).get() / congestion);
        ProblemData::Depot const &end = data.location(trip->endDepot());

        ds = DurationSegment::merge(edgeDuration, ds, {end});
        ds = ds.finaliseBack();
    }

    duration_ = ds.duration();
    durationCost_ = vehData.unitDurationCost * static_cast<Cost>(duration_);
    startTime_ = ds.startEarly();
    slack_ = ds.slack();
    timeWarp_ = ds.timeWarp(vehData.maxDuration);

    makeSchedule(data);
}

Route::Route(Trips trips,
             Distance distance,
             Cost distanceCost,
             Distance excessDistance,
             std::vector<Load> delivery,
             std::vector<Load> pickup,
             std::vector<Load> excessLoad,
             Duration duration,
             Cost durationCost,
             Duration timeWarp,
             Duration travel,
             Duration service,
             Duration startTime,
             Duration slack,
             Cost prizes,
             std::pair<double, double> centroid,
             size_t vehicleType,
             size_t startDepot,
             size_t endDepot,
             std::vector<ScheduledVisit> schedule)
    : trips_(std::move(trips)),
      schedule_(std::move(schedule)),
      distance_(distance),
      distanceCost_(distanceCost),
      excessDistance_(excessDistance),
      delivery_(std::move(delivery)),
      pickup_(std::move(pickup)),
      excessLoad_(std::move(excessLoad)),
      duration_(duration),
      durationCost_(durationCost),
      timeWarp_(timeWarp),
      travel_(travel),
      service_(service),
      startTime_(startTime),
      slack_(slack),
      prizes_(prizes),
      centroid_(centroid),
      vehicleType_(vehicleType),
      startDepot_(startDepot),
      endDepot_(endDepot)
{
}

bool Route::empty() const { return size() == 0; }

size_t Route::size() const
{
    return std::accumulate(trips_.begin(),
                           trips_.end(),
                           0,
                           [](size_t count, auto const &trip)
                           { return count + trip.size(); });
}

size_t Route::numTrips() const { return trips_.size(); }

Client Route::operator[](size_t idx) const
{
    for (auto const &trip : trips_)
        if (idx < trip.size())
            return trip[idx];
        else
            idx -= trip.size();

    throw std::out_of_range("Index out of range.");
}

Route::Iterator Route::begin() const { return Iterator(*this, 0); }

Route::Iterator Route::end() const { return Iterator(*this, size()); }

Route::Trips const &Route::trips() const { return trips_; }

Trip const &Route::trip(size_t idx) const
{
    assert(idx < trips_.size());
    return trips_[idx];
}

Route::Visits Route::visits() const { return {begin(), end()}; }

std::vector<Route::ScheduledVisit> const &Route::schedule() const
{
    return schedule_;
}

Distance Route::distance() const { return distance_; }

Cost Route::distanceCost() const { return distanceCost_; }

Distance Route::excessDistance() const { return excessDistance_; }

std::vector<Load> const &Route::delivery() const { return delivery_; }

std::vector<Load> const &Route::pickup() const { return pickup_; }

std::vector<Load> const &Route::excessLoad() const { return excessLoad_; }

Duration Route::duration() const { return duration_; }

Cost Route::durationCost() const { return durationCost_; }

Duration Route::serviceDuration() const { return service_; }

Duration Route::timeWarp() const { return timeWarp_; }

Duration Route::waitDuration() const { return duration_ - travel_ - service_; }

Duration Route::travelDuration() const { return travel_; }

Duration Route::startTime() const { return startTime_; }

Duration Route::endTime() const { return startTime_ + duration_ - timeWarp_; }

Duration Route::slack() const { return slack_; }

Duration Route::releaseTime() const { return trips_[0].releaseTime(); }

Cost Route::prizes() const { return prizes_; }

std::pair<double, double> const &Route::centroid() const { return centroid_; }

size_t Route::vehicleType() const { return vehicleType_; }

size_t Route::startDepot() const { return startDepot_; }

size_t Route::endDepot() const { return endDepot_; }

bool Route::isFeasible() const
{
    return !hasExcessLoad() && !hasTimeWarp() && !hasExcessDistance();
}

bool Route::hasExcessLoad() const
{
    return std::any_of(excessLoad_.begin(),
                       excessLoad_.end(),
                       [](auto const excess) { return excess > 0; });
}

bool Route::hasExcessDistance() const { return excessDistance_ > 0; }

bool Route::hasTimeWarp() const { return timeWarp_ > 0; }

double pyvrp::Route::fuelAndEmissionCostWithConstantVelocityConstantCongestion(
    ProblemData const &data) const
{
    Duration duration = this->duration();
    ProblemData::VehicleType vehicleType
        = data.vehicleType(this->vehicleType());
    double vehicleWeightInTons
        = vehicleType.vehicleWeight / 1000.0;        // convert to tons
    double durationInHours = duration.get() / 3600;  // convert to hours
    double emissionFactor
        = pyvrp::utils::emissionCostPerTonPerHourConstantVelocity(
              vehicleType.powerToMassRatio,
              vehicleType.velocity * vehicleType.congestion)
          * vehicleWeightInTons * durationInHours;

    double fuelAndEmissionCost
        = (vehicleType.unitFuelCost + vehicleType.unitEmissionCost)
          * emissionFactor;

    return fuelAndEmissionCost;
}

/**
 * Calculate fuel and emission cost for a route by breaking it down into
 * individual segments. For each segment, it is assumed that the velocity is
 * constant and it is calculated by getting the distance of the segment and the
 * duration to complete the segment.
 * Note that the route must be associated with a vehicle that has the vehicle
 * weight and power to mass ratio already defined. Otherwise, the value will be
 * 0.
 */
double pyvrp::Route::
    fuelAndEmissionCostWithConstantVelocityInSegmentsConstantCongestion(
        ProblemData const &data) const
{
    double costForRoute = 0;
    auto vehicleType = data.vehicleType(this->vehicleType());
    double vehicleWeightInTons
        = vehicleType.vehicleWeight / 1000.0;  // convert to tons
    auto distanceMatrix = data.distanceMatrix(vehicleType.profile);
    auto durationMatrix = data.durationMatrix(vehicleType.profile);

    for (Trip trip : this->trips())
    {
        size_t from = trip.startDepot();
        size_t to;

        for (size_t client : trip.visits())
        {
            size_t to = client;

            Distance distanceOfSegment = distanceMatrix(from, to);
            Duration durationOfSegment = durationMatrix(from, to);
            if (durationOfSegment == 0)
            {
                // std::cout
                //     << "Duration is 0. Velocity cannot be infinite. Did you "
                //        "provide a duration matrix? And if you did, check if "
                //        "the "
                //        "duration between client: "
                //            + std::to_string(from)
                //            + " and client: " + std::to_string(to) + " is not
                //            0."
                //     << std::endl;
                from = to;
                continue;  // continue as the cost for going from the node to
                           // itself is always 0.
            };
            double velocityInSegment
                = distanceOfSegment.get() / durationOfSegment.get();
            double durationOfSegmentInHours
                = distanceOfSegment.get() / 3600;  // convert to hours
            double emissionFactor
                = pyvrp::utils::emissionCostPerTonPerHourConstantVelocity(
                      vehicleType.powerToMassRatio,
                      velocityInSegment * vehicleType.congestion)
                  * vehicleWeightInTons * durationOfSegmentInHours;

            double fuelAndEmissionCost
                = (vehicleType.unitFuelCost + vehicleType.unitEmissionCost)
                  * emissionFactor;

            costForRoute += fuelAndEmissionCost;
            from = to;
        }
        to = trip.endDepot();
        Distance distanceOfSegment = distanceMatrix(from, to);
        Duration durationOfSegment = durationMatrix(from, to);
        if (durationOfSegment == 0)
        {
            // std::cout
            //     << "Duration is 0. Velocity cannot be infinite. Did you "
            //        "provide a duration matrix? And if you did, check if the "
            //        "duration between client: "
            //            + std::to_string(from)
            //            + " and client: " + std::to_string(to) + " is not 0."
            //     << std::endl;
            continue;  // continue as the cost for going from the node to itself
                       // is always 0.
        };
        double velocityInSegment
            = distanceOfSegment.get() / durationOfSegment.get();
        double durationOfSegmentInHours
            = distanceOfSegment.get() / 3600;  // convert to hours
        double emissionFactor
            = pyvrp::utils::emissionCostPerTonPerHourConstantVelocity(
                  vehicleType.powerToMassRatio,
                  velocityInSegment * vehicleType.congestion)
              * vehicleWeightInTons * durationOfSegmentInHours;

        double fuelAndEmissionCost
            = (vehicleType.unitFuelCost + vehicleType.unitEmissionCost)
              * emissionFactor;

        costForRoute += fuelAndEmissionCost;
    }

    return costForRoute;
}

double pyvrp::Route::fuelAndEmissionCostWithNonLinearVelocityConstantCongestion(
    ProblemData const &data) const
{
    double costForRoute = 0;
    auto vehicleType = data.vehicleType(this->vehicleType());
    double vehicleWeightInTons
        = vehicleType.vehicleWeight / 1000.0;  // convert to tons
    auto distanceMatrix = data.distanceMatrix(vehicleType.profile);
    auto durationMatrix = data.durationMatrix(vehicleType.profile);

    for (Trip trip : this->trips())
    {
        size_t from = trip.startDepot();
        size_t to;

        for (size_t client : trip.visits())
        {
            size_t to = client;
            Distance distanceOfSegment = distanceMatrix(from, to);
            Duration durationOfSegment = durationMatrix(from, to);
            if (durationOfSegment == 0)
            {
                // std::cout
                //     << "Duration is 0. Velocity cannot be infinite. Did you "
                //        "provide a duration matrix? And if you did, check if "
                //        "the "
                //        "duration between client: "
                //            + std::to_string(from)
                //            + " and client: " + std::to_string(to) + " is not
                //            0."
                //     << std::endl;
                from = to;
                continue;  // continue as the cost for going from the node to
                           // itself is always 0.
            };
            pyvrp::velocity::WLTCProfile velocityProfile
                = pyvrp::velocity::getProfileBasedOnDistance(
                    distanceOfSegment.get());
            double emissionFactor
                = pyvrp::utils::emissionFactorPerTonNonLinearVelocity(
                      vehicleType.powerToMassRatio,
                      vehicleType.congestion,
                      durationOfSegment.get(),
                      distanceOfSegment.get(),
                      velocityProfile.getSquaredVelocityIntegral(
                          durationOfSegment.get()),
                      velocityProfile.getCubedVelocityIntegral(
                          durationOfSegment.get()))
                  * vehicleWeightInTons;

            double fuelAndEmissionCost
                = (vehicleType.unitFuelCost + vehicleType.unitEmissionCost)
                  * emissionFactor;

            costForRoute += fuelAndEmissionCost;
            from = to;
        }
        to = trip.endDepot();
        Distance distanceOfSegment = distanceMatrix(from, to);
        Duration durationOfSegment = durationMatrix(from, to);
        if (durationOfSegment == 0)
        {
            // std::cout
            //     << "Duration is 0. Velocity cannot be infinite. Did you "
            //        "provide a duration matrix? And if you did, check if the "
            //        "duration between client: "
            //            + std::to_string(from)
            //            + " and client: " + std::to_string(to) + " is not 0."
            //     << std::endl;
            continue;  // continue as the cost for going from the node to itself
                       // is always 0.
        };
        pyvrp::velocity::WLTCProfile velocityProfile
            = pyvrp::velocity::getProfileBasedOnDistance(
                distanceOfSegment.get());
        double emissionFactor
            = pyvrp::utils::emissionFactorPerTonNonLinearVelocity(
                  vehicleType.powerToMassRatio,
                  vehicleType.congestion,
                  durationOfSegment.get(),
                  distanceOfSegment.get(),
                  velocityProfile.getSquaredVelocityIntegral(
                      durationOfSegment.get()),
                  velocityProfile.getCubedVelocityIntegral(
                      durationOfSegment.get()))
              * vehicleWeightInTons;

        double fuelAndEmissionCost
            = (vehicleType.unitFuelCost + vehicleType.unitEmissionCost)
              * emissionFactor;

        costForRoute += fuelAndEmissionCost;
    }
    return costForRoute;
}

double pyvrp::Route::
    fuelAndEmissionCostWithConstantVelocityConstantCongestionInSegments(
        [[maybe_unused]] ProblemData const &data) const
{
    // TODO: Implement this function.
    return 0.0;
}

double pyvrp::Route::
    fuelAndEmissionCostWithConstantVelocityInSegmentsConstantCongestionInSegments(
        [[maybe_unused]] ProblemData const &data) const
{
    // TODO: Implement this function.
    return 0.0;
}

double pyvrp::Route::
    fuelAndEmissionCostWithNonLinearVelocityConstantCongestionInSegments(
        [[maybe_unused]] ProblemData const &data) const
{
    // TODO: Implement this function.
    return 0.0;
}

double pyvrp::Route::fuelAndEmissionCostWithConstantVelocityNonLinearCongestion(
    [[maybe_unused]] ProblemData const &data) const
{
    // TODO: Implement this function.
    return 0.0;
}

double pyvrp::Route::
    fuelAndEmissionCostWithConstantVelocityInSegmentsNonLinearCongestion(
        [[maybe_unused]] ProblemData const &data) const
{
    // TODO: Implement this function.
    return 0.0;
}

double
pyvrp::Route::fuelAndEmissionCostWithNonLinearVelocityNonLinearCongestion(
    [[maybe_unused]] ProblemData const &data) const
{
    // TODO: Implement this function.
    return 0.0;
}

double pyvrp::Route::fuelAndEmissionCost(ProblemData const &data) const
{
    if (data.velocityBehaviour()
            == pyvrp::velocity::VelocityBehaviour::ConstantVelocity
        && data.congestionBehaviour()
               == pyvrp::congestion::CongestionBehaviour::ConstantCongestion)
    {
        return fuelAndEmissionCostWithConstantVelocityConstantCongestion(data);
    }
    else if (
        data.velocityBehaviour()
            == pyvrp::velocity::VelocityBehaviour::ConstantVelocityInSegment
        && data.congestionBehaviour()
               == pyvrp::congestion::CongestionBehaviour::ConstantCongestion)
    {
        return fuelAndEmissionCostWithConstantVelocityInSegmentsConstantCongestion(
            data);
    }
    else if (data.velocityBehaviour()
                 == pyvrp::velocity::VelocityBehaviour::VariableVelocity
             && data.congestionBehaviour()
                    == pyvrp::congestion::CongestionBehaviour::
                        ConstantCongestion)
    {
        return fuelAndEmissionCostWithNonLinearVelocityConstantCongestion(data);
    }
    else if (data.velocityBehaviour()
                 == pyvrp::velocity::VelocityBehaviour::ConstantVelocity
             && data.congestionBehaviour()
                    == pyvrp::congestion::CongestionBehaviour::
                        ConstantCongestionInSegment)
    {
        return fuelAndEmissionCostWithConstantVelocityConstantCongestionInSegments(
            data);
    }
    else if (data.velocityBehaviour()
                 == pyvrp::velocity::VelocityBehaviour::
                     ConstantVelocityInSegment
             && data.congestionBehaviour()
                    == pyvrp::congestion::CongestionBehaviour::
                        ConstantCongestionInSegment)
    {
        return fuelAndEmissionCostWithConstantVelocityInSegmentsConstantCongestionInSegments(
            data);
    }
    else if (data.velocityBehaviour()
                 == pyvrp::velocity::VelocityBehaviour::VariableVelocity
             && data.congestionBehaviour()
                    == pyvrp::congestion::CongestionBehaviour::
                        ConstantCongestionInSegment)
    {
        return fuelAndEmissionCostWithNonLinearVelocityConstantCongestionInSegments(
            data);
    }
    else if (data.velocityBehaviour()
                 == pyvrp::velocity::VelocityBehaviour::ConstantVelocity
             && data.congestionBehaviour()
                    == pyvrp::congestion::CongestionBehaviour::
                        VariableCongestion)
    {
        return fuelAndEmissionCostWithConstantVelocityNonLinearCongestion(data);
    }
    else if (
        data.velocityBehaviour()
            == pyvrp::velocity::VelocityBehaviour::ConstantVelocityInSegment
        && data.congestionBehaviour()
               == pyvrp::congestion::CongestionBehaviour::VariableCongestion)
    {
        return fuelAndEmissionCostWithConstantVelocityInSegmentsNonLinearCongestion(
            data);
    }
    else if (data.velocityBehaviour()
                 == pyvrp::velocity::VelocityBehaviour::VariableVelocity
             && data.congestionBehaviour()
                    == pyvrp::congestion::CongestionBehaviour::
                        VariableCongestion)
    {
        return fuelAndEmissionCostWithNonLinearVelocityNonLinearCongestion(
            data);
    }
    else
    {
        throw std::runtime_error(
            "Unknown velocity and congestion behaviour combination.");
    }
}

double pyvrp::Route::wageCost(ProblemData const &data) const
{
    pyvrp::ProblemData::VehicleType const vehicleType
        = data.vehicleType(this->vehicleType());
    auto const hoursPaid = std::max(this->duration(), vehicleType.minHoursPaid);
    return hoursPaid.get() * vehicleType.wagePerHour.get();
}

bool Route::operator==(Route const &other) const
{
    // First compare simple attributes, since that's a quick and cheap check.
    // Only when these are the same we test if the visits are all equal.
    // clang-format off
    return distance_ == other.distance_
        && duration_ == other.duration_
        && timeWarp_ == other.timeWarp_
        && vehicleType_ == other.vehicleType_
        && trips_ == other.trips_;
    // clang-format on
}

std::ostream &operator<<(std::ostream &out, Route const &route)
{
    auto const &trips = route.trips();
    for (size_t idx = 0; idx != trips.size(); ++idx)
    {
        if (idx != 0)
            out << " | ";
        out << trips[idx];
    }

    return out;
}
