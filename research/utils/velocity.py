from datetime import time
from pathlib import Path

import pandas as pd
from scipy.interpolate import CubicSpline

WLTC_DIR = Path(__file__).parent.parent / "data" / "wltc"
SLOW_PROFILE = WLTC_DIR / "class3-1-Low.csv"
MEDIUM_PROFILE = WLTC_DIR / "class3-2a-Medium.csv"
HIGH_PROFILE = WLTC_DIR / "class3-3a-High.csv"

SLOW_PROFILE_DATA = pd.read_csv(SLOW_PROFILE, names=["time", "speed"])
SLOW_PROFILE_DATA["speed"] = SLOW_PROFILE_DATA["speed"] * 1000 / 3600  # Convert to m/s
full_slow_profile = SLOW_PROFILE_DATA.copy(deep=True)
full_slow_profile["time"] -= full_slow_profile.loc[:, "time"].iloc[0]


MEDIUM_PROFILE_DATA = pd.read_csv(MEDIUM_PROFILE, names=["time", "speed"])
MEDIUM_PROFILE_DATA["speed"] = (
    MEDIUM_PROFILE_DATA["speed"] * 1000 / 3600
)  # Convert to m/s
full_medium_profile = MEDIUM_PROFILE_DATA.copy(deep=True)
full_medium_profile["time"] -= full_medium_profile.loc[:, "time"].iloc[0]

# Filter the medium profile data to only include the relevant time range
MEDIUM_PROFILE_DATA = MEDIUM_PROFILE_DATA[
    (600 <= MEDIUM_PROFILE_DATA["time"]) & (MEDIUM_PROFILE_DATA["time"] <= 986)
]
MEDIUM_PROFILE_DATA["time"] -= MEDIUM_PROFILE_DATA.loc[:, "time"].iloc[0]


HIGH_PROFILE_DATA = pd.read_csv(HIGH_PROFILE, names=["time", "speed"])
HIGH_PROFILE_DATA["speed"] = HIGH_PROFILE_DATA["speed"] * 1000 / 3600  # Convert to m/s
full_high_profile = HIGH_PROFILE_DATA.copy(deep=True)
full_high_profile["time"] -= full_high_profile.loc[:, "time"].iloc[0]

# Filter the high profile data to only include the relevant time range
HIGH_PROFILE_DATA = HIGH_PROFILE_DATA[
    (1093 <= HIGH_PROFILE_DATA["time"]) & (HIGH_PROFILE_DATA["time"] <= 1364)
]
HIGH_PROFILE_DATA["time"] -= HIGH_PROFILE_DATA.loc[:, "time"].iloc[0]


full_slow_profile_spline = CubicSpline(
    x=full_slow_profile["time"], y=full_slow_profile["speed"]
)
full_medium_profile_spline = CubicSpline(
    x=full_medium_profile["time"], y=full_medium_profile["speed"]
)
full_high_profile_spline = CubicSpline(
    x=full_high_profile["time"], y=full_high_profile["speed"]
)


def get_spline_for_profile(
    profile: str, congestion_factor: float = 1, power: int = 1
) -> CubicSpline:
    if profile == "slow":
        return CubicSpline(
            x=SLOW_PROFILE_DATA["time"],
            y=(SLOW_PROFILE_DATA["speed"] * congestion_factor) ** power,
        )
    elif profile == "medium":
        return CubicSpline(
            x=MEDIUM_PROFILE_DATA["time"],
            y=(MEDIUM_PROFILE_DATA["speed"] * congestion_factor) ** power,
        )
    else:
        return CubicSpline(
            x=HIGH_PROFILE_DATA["time"],
            y=(HIGH_PROFILE_DATA["speed"] * congestion_factor) ** power,
        )


def get_congestion_factor(time_of_day: time) -> float:
    if time_of_day < time(7, 0):
        return 1.0  # Off-peak hours
    elif time_of_day < time(9, 0):
        return 1.5  # Morning rush hour
    elif time_of_day < time(17, 0):
        return 1.0  # Daytime
    else:
        return 2.0  # Evening rush hour
