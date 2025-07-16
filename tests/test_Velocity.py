from pathlib import Path
from typing import Iterator

import pandas as pd
import pytest
from numpy.testing import assert_allclose

from pyvrp import WLTCProfile, get_profile_based_on_distance

SLOW_PROFILE_DISTANCE = 1_000
MEDIUM_PROFILE_DISTANCE = 10_000
HIGH_PROFILE_DISTANCE = 100_000


@pytest.mark.parametrize(
    "distance, expected_profile",
    [
        (SLOW_PROFILE_DISTANCE, "slow"),
        (MEDIUM_PROFILE_DISTANCE, "medium"),
        (HIGH_PROFILE_DISTANCE, "high"),
    ],
)
def test_get_profile_based_on_distance(
    distance: float, expected_profile: str
) -> None:
    profile = get_profile_based_on_distance(distance)
    assert profile.name == expected_profile


class TestSlowProfile:
    @pytest.fixture(scope="function")
    def slow_profile(self) -> Iterator[WLTCProfile]:
        yield get_profile_based_on_distance(SLOW_PROFILE_DISTANCE)

    @pytest.mark.parametrize(
        "time, expected_distance",
        [
            (10, 0),
            (50, 300.55577),
            (100, 614.04798),
        ],
    )
    def test_get_distance_for_travel_time(
        self, slow_profile: WLTCProfile, time: float, expected_distance: float
    ) -> None:
        assert_allclose(
            slow_profile.get_distance_for_travel_time(time),
            expected_distance,
            atol=1e-3,
        )

    @pytest.mark.parametrize(
        "time, expected_value",
        [
            (10, 0),
            (50, 2778.51805),
            (100, 5278.06570),
        ],
    )
    def test_get_squared_velocity_integral(
        self, slow_profile: WLTCProfile, time: float, expected_value: float
    ) -> None:
        assert_allclose(
            slow_profile.get_squared_velocity_integral(time),
            expected_value,
            atol=1e-3,
        )

    @pytest.mark.parametrize(
        "time, expected_value",
        [
            (10, 0),
            (50, 27720.59425),
            (100, 50071.75082),
        ],
    )
    def test_get_cubed_velocity_integral(
        self, slow_profile: WLTCProfile, time: float, expected_value: float
    ) -> None:
        assert_allclose(
            slow_profile.get_cubed_velocity_integral(time),
            expected_value,
            atol=1e-3,
        )


class TestMediumProfile:
    @pytest.fixture(scope="function")
    def medium_profile(self) -> Iterator[WLTCProfile]:
        yield get_profile_based_on_distance(MEDIUM_PROFILE_DISTANCE)

    @pytest.mark.parametrize(
        "time, expected_distance",
        [
            (10, 39.72550),
            (50, 582.20125),
            (100, 1010.59966),
        ],
    )
    def test_get_distance_for_travel_time(
        self,
        medium_profile: WLTCProfile,
        time: float,
        expected_distance: float,
    ) -> None:
        assert_allclose(
            medium_profile.get_distance_for_travel_time(time),
            expected_distance,
            atol=1e-3,
        )

    @pytest.mark.parametrize(
        "time, expected_value",
        [
            (10, 245.26706),
            (50, 7685.504593),
            (100, 11815.23467),
        ],
    )
    def test_get_squared_velocity_integral(
        self, medium_profile: WLTCProfile, time: float, expected_value: float
    ) -> None:
        assert_allclose(
            medium_profile.get_squared_velocity_integral(time),
            expected_value,
            atol=1e-3,
        )

    @pytest.mark.parametrize(
        "time, expected_value",
        [
            (10, 1712.008071),
            (50, 104822.04152),
            (100, 148663.32623),
        ],
    )
    def test_get_cubed_velocity_integral(
        self, medium_profile: WLTCProfile, time: float, expected_value: float
    ) -> None:
        assert_allclose(
            medium_profile.get_cubed_velocity_integral(time),
            expected_value,
            atol=1e-3,
        )


class TestHighProfile:
    @pytest.fixture(scope="function")
    def high_profile(self) -> Iterator[WLTCProfile]:
        yield get_profile_based_on_distance(HIGH_PROFILE_DISTANCE)

    @pytest.mark.parametrize(
        "time, expected_distance",
        [
            (10, 170.89023),
            (50, 694.79117),
            (100, 1426.80286),
        ],
    )
    def test_get_distance_for_travel_time(
        self, high_profile: WLTCProfile, time: float, expected_distance: float
    ) -> None:
        assert_allclose(
            high_profile.get_distance_for_travel_time(time),
            expected_distance,
            atol=1e-3,
        )

    @pytest.mark.parametrize(
        "time, expected_value",
        [
            (10, 2923.42556),
            (50, 10999.34522),
            (100, 23952.99346),
        ],
    )
    def test_get_squared_velocity_integral(
        self, high_profile: WLTCProfile, time: float, expected_value: float
    ) -> None:
        assert_allclose(
            high_profile.get_squared_velocity_integral(time),
            expected_value,
            atol=1e-3,
        )

    @pytest.mark.parametrize(
        "time, expected_value",
        [
            (10, 50063.11752),
            (50, 183219.15437),
            (100, 431938.80765),
        ],
    )
    def test_get_cubed_velocity_integral(
        self, high_profile: WLTCProfile, time: float, expected_value: float
    ) -> None:
        assert_allclose(
            high_profile.get_cubed_velocity_integral(time),
            expected_value,
            atol=1e-3,
        )


class TestWLTCProfile:
    """
    We use v(T) = T where T is hours. To convert the problem into seconds, we must do some adjustments.
    That eventually leads to T = t / 3600 and thus, we can drop this substition directly into the
    integral solution.

    Finding the integral of v, v^2, and v^3, we would have:
    integral(v(t)dt) = (1/2) * (t/3600)^2
    integral(v^2(t)dt) = (1/3) * (t/3600)^3
    integral(v^3(t)dt) = (1/4) * (t/3600)^4
    We assume for all cases, v(0) = 0, thus, the constant term is ignored.
    """

    @pytest.fixture(scope="function")
    def dummy_csv_path(self, tmp_path: Path) -> Iterator[Path]:
        path = tmp_path / "data.csv"
        yield path
        if path.exists():
            path.unlink()

    @pytest.fixture(scope="function")
    def dummy_csv(self, dummy_csv_path: Path) -> Iterator[pd.DataFrame]:
        df = pd.DataFrame(
            {
                "time": [0, 1, 2, 3, 4, 5],  # in hours
                "velocity": [0, 1, 2, 3, 4, 5],  # in kmph
            }
        )
        df["time"] *= 3600  # convert to seconds
        # Conversion to m/s happens in the WLTCProfile class

        df.to_csv(dummy_csv_path, index=False)

        yield df

    @pytest.fixture(scope="function")
    def dummy_wltc_profile(
        self, dummy_csv: pd.DataFrame, dummy_csv_path: Path
    ) -> Iterator[WLTCProfile]:
        yield WLTCProfile("test", dummy_csv_path.absolute().as_posix(), 0, 0)

    @pytest.mark.parametrize("time", [0, 1800, 3600, 5400, 7200, 9000])
    def test_get_distance_for_travel_time(
        self,
        dummy_wltc_profile: WLTCProfile,
        time: float,
    ) -> None:
        expected_value = ((time / 3600) ** 2) / 2 * 1000  # in meters
        assert_allclose(
            dummy_wltc_profile.get_distance_for_travel_time(time),
            expected_value,
        )

    @pytest.mark.parametrize("time", [0, 1800, 3600, 5400, 7200, 9000])
    def test_get_squared_velocity_integral(
        self, dummy_wltc_profile: WLTCProfile, time: float
    ) -> None:
        expected_value = (
            ((time / 3600) ** 3) / 3 * 1000 * 1000 / 3600
        )  # Convert to m^2/s
        assert_allclose(
            dummy_wltc_profile.get_squared_velocity_integral(time),
            expected_value,
            atol=1e-3,
        )

    @pytest.mark.parametrize("time", [0, 1800, 3600, 5400, 7200, 9000])
    def test_get_cubed_velocity_integral(
        self, dummy_wltc_profile: WLTCProfile, time: float
    ) -> None:
        expected_value = (
            ((time / 3600) ** 4) / 4 * 1000 * 1000 * 1000 / 3600 / 3600
        )  # Convert to m^3/s^2
        assert_allclose(
            dummy_wltc_profile.get_cubed_velocity_integral(time),
            expected_value,
            atol=1e-3,
        )

    @pytest.mark.parametrize("distance", [0, 125, 500, 1125, 2000, 3125])
    def test_get_time_from_travel_distance(
        self, dummy_wltc_profile: WLTCProfile, distance: float
    ) -> None:
        expected_value = ((2 * (distance / 1000)) ** 0.5) * 3600
        assert_allclose(
            dummy_wltc_profile.get_time_for_travel_distance(distance),
            expected_value,
            atol=1e-6,
        )
