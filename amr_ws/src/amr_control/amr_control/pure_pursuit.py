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
        closest_xy = (0.0, 0.0)
        closest_idx = 0
        closest_distance = float("inf")

        for i, (path_x, path_y) in enumerate(self._path):
            distance = abs(x - path_x) + abs(y - path_y)
            if distance < closest_distance:
                closest_distance = distance
                closest_xy = (path_x, path_y)
                closest_idx = i

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
        target_xy = (0.0, 0.0)
        return target_xy
