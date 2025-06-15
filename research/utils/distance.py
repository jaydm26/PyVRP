from research.utils.types import TCoordinate


def calculate_distance(coord1: TCoordinate, coord2: TCoordinate) -> float:
    """
    Distance is given by the average (expected) distance between two points in a 2D space.
    Distance will be between the Euclidean distance and the Manhattan distance.

    The formula used is:
    d = sqrt((x1 - x2)^2 + (y1 - y2)^2) * ((1 + sqrt(2)) / 2)

    where (x1, y1) and (x2, y2) are the coordinates of the two points.
    """
    euclidean_distance = (
        (coord1[0] - coord2[0]) ** 2 + (coord1[1] - coord2[1]) ** 2
    ) ** 0.5
    multiplier = (
        1 + 2**0.5
    ) / 2  # <-- This multiplier is used to adjust the distance as per the problem requirements.
    return euclidean_distance * multiplier
