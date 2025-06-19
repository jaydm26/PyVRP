from pyvrp._pyvrp import get_profile_based_on_distance


def get_time_from_distance(distance: int) -> int:
    velocity_profile = get_profile_based_on_distance(distance)

    duration = velocity_profile.get_time_for_travel_distance(distance)
    return round(duration)
