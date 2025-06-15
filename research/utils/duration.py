from scipy.integrate import quad
from scipy.optimize import root_scalar
from scipy.interpolate import CubicSpline


from research.utils.velocity import (
    SLOW_PROFILE_DATA,
    MEDIUM_PROFILE_DATA,
    HIGH_PROFILE_DATA,
    full_slow_profile,
    full_medium_profile,
    full_slow_profile_spline,
    full_medium_profile_spline,
    full_high_profile_spline,
    get_spline_for_profile,
)

full_distance_for_slow = quad(
    full_slow_profile_spline,
    full_slow_profile["time"].min(),
    full_slow_profile["time"].max(),
    limit=full_slow_profile.shape[0],
)[0]
MAX_SLOW_DISTANCE = 3 * full_distance_for_slow
full_distance_for_medium = quad(
    full_medium_profile_spline,
    full_medium_profile["time"].min(),
    full_medium_profile["time"].max(),
    limit=full_medium_profile.shape[0],
)[0]
MAX_MEDIUM_DISTANCE = 3 * full_distance_for_medium


def integrate_to_time(
    target_time: float, target_distance: float, spline: CubicSpline, x0: float
) -> float:
    result, _ = quad(spline, x0, target_time)
    return result - target_distance


def get_duration_for_distance(target_distance: float) -> float:
    # There is some logic needed for the the high velocity profile as it is slightly convoluted in the set up.
    total_elapsed_time = 0.0

    if target_distance <= MAX_SLOW_DISTANCE:
        spline = get_spline_for_profile("slow")
        t0, t1 = SLOW_PROFILE_DATA["time"].min(), SLOW_PROFILE_DATA["time"].max()
        repeatable_distance = full_distance_for_slow
    elif target_distance <= MAX_MEDIUM_DISTANCE:
        spline = get_spline_for_profile("medium")
        t0, t1 = MEDIUM_PROFILE_DATA["time"].min(), MEDIUM_PROFILE_DATA["time"].max()
        repeatable_distance = quad(spline, t0, t1)[0]
    else:
        spline = get_spline_for_profile("high")
        # Assumption is that the distance travelled in high profile is higher than the full distance of the profile.
        t0, t1 = HIGH_PROFILE_DATA["time"].min(), HIGH_PROFILE_DATA["time"].max()
        repeatable_distance = quad(spline, t0, t1)[0]
        # The first 70 seconds are the initial acceleration, and the next 113 seconds are the deceleration.
        # We exclude the initial 4 seconds and the last 26 seconds from the total time to exclude no movement.
        total_elapsed_time += (70 - 4) + (113 - 26)
        # Reduce the target distance by the distance travelled in the first and last segments.
        # The first segment is from 0 to 70 seconds, and the second segment is from 341 to 454 seconds.
        # The distance travelled in these segments is not part of the repeatable distance.
        target_distance -= (
            quad(full_high_profile_spline, 0, 70)[0]
            + quad(full_high_profile_spline, 341, 454)[0]
        )

    repeated_profiles = target_distance // repeatable_distance
    remaining_distance = target_distance - (repeatable_distance * repeated_profiles)
    total_elapsed_time += t1 * repeated_profiles

    result = root_scalar(
        integrate_to_time,
        args=(remaining_distance, spline, t0),
        bracket=[t0, t1],
    )
    total_elapsed_time += result.root
    return total_elapsed_time


if __name__ == "__main__":
    i = 55.66
    print(
        f"Distance: {i} km, Time: {get_duration_for_distance(i * 1000) / 60 / 60:.2f} min"
    )
