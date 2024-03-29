import math
from typing import List, Tuple


class PurePursuit:
    """Class to follow a path using a simple pure pursuit controller."""

    def __init__(self, dt: float, lookahead_distance: float = 0.5):
        """Pure pursuit class initializer.

        Args:
            dt: Sampling period [s].
            lookahead_distance: Distance to the next target point [m].

        """
        self._dt: float = dt
        self._lookahead_distance: float = lookahead_distance
        self._path: List[Tuple[float, float]] = []

    def compute_commands(self, x: float, y: float, theta: float) -> Tuple[float, float]:
        """Pure pursuit controller implementation.

        Args:
            x: Estimated robot x coordinate [m].
            y: Estimated robot y coordinate [m].
            theta: Estimated robot heading [rad].

        Returns:
            v: Linear velocity [m/s].
            w: Angular velocity [rad/s].

        """
        # TODO: 4.4. Complete the function body with your code (i.e., compute v and w).
        try:
            # 1. Find the closest path point to the current robot pose.
            closest_xy, closest_idx = self._find_closest_point(x, y)
            # given orientation of the robot check if looking away

            """ path_orientation = math.atan2(closest_xy[1] - y, closest_xy[0] - x)
            starting = True
            looking_away = True
            if starting and not abs(path_orientation - theta) > math.pi / 2:
                looking_away = False
                starting = False """

            """ if looking_away:
                v = 0.0
                w = 1.0
            else: """
            # 2. Find the target point based on the lookahead distance.
            (target_x, target_y) = self._find_target_point(closest_xy, closest_idx)
            v = 0.6  # Linear velocity [m/s].
            alpha = math.atan2(target_y - y, target_x - x) - theta
            w = 2 * v * math.sin(alpha) / self._lookahead_distance

        except ValueError:
            print("ValueError344")
            v = 0.0
            w = 0.0

        return v, w

    @property
    def path(self) -> List[Tuple[float, float]]:
        """Path getter."""
        return self._path

    @path.setter
    def path(self, value: List[Tuple[float, float]]) -> None:
        """Path setter."""
        self._path = value

    def _find_closest_point(self, x: float, y: float) -> Tuple[Tuple[float, float], int]:
        """Find the closest path point to the current robot pose.

        Args:
            x: Estimated robot x coordinate [m].
            y: Estimated robot y coordinate [m].

        Returns:
            Tuple[float, float]: (x, y) coordinates of the closest path point [m].
            int: Index of the path point found.

        """
        # TODO: 4.2. Complete the function body (i.e., find closest_xy and closest_idx).
        closest_xy = (0.0, 0.0)  # Closest path point (x, y) [m].
        closest_idx = 0  # Index of the closest path point.
        # 1. Iterate over the path points to find the closest one.
        distances = []
        for i, (path_x, path_y) in enumerate(self._path):
            # 2. Compute the distance between the robot and the current path point.
            x_diff = x - path_x
            y_diff = y - path_y
            distance = abs(x_diff) + abs(y_diff)
            distances.append(distance)
        # 3. Find the index of the closest path point.
        closest_idx = distances.index(min(distances))
        closest_xy = self._path[closest_idx]
        return closest_xy, closest_idx

    def _find_target_point(
        self, origin_xy: Tuple[float, float], origin_idx: int
    ) -> Tuple[float, float]:
        """Find the destination path point based on the lookahead distance.

        Args:
            origin_xy: Current location of the robot (x, y) [m].
            origin_idx: Index of the current path point.

        Returns:
            Tuple[float, float]: (x, y) coordinates of the target point [m].

        """
        # TODO: 4.3. Complete the function body with your code (i.e., determine target_xy).
        target_xy = (0.0, 0.0)  # Target point (x, y) [m] intialized to the origin.
        # Look for points ahead of the current origin_idx
        # Compute the distance between the robot and each path point
        for i, (path_x, path_y) in enumerate(self._path[origin_idx:]):
            x_diff = origin_xy[0] - path_x
            y_diff = origin_xy[1] - path_y
            distance = (x_diff**2 + y_diff**2) ** 0.5
            # If the distance is greater than the lookahead distance, the target point is found
            if distance > self._lookahead_distance:
                target_xy = (path_x, path_y)
                break
            # Make sure that if the last point of the path (goal) is closer than the look ahead
            # distance, the target point is the last point of the path
            if i == len(self._path[origin_idx:]) - 1:
                target_xy = (path_x, path_y)

        return target_xy
