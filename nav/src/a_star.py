from typing import Dict, Optional, List

import numpy


class AStar:
    """
    The implementation of the A* algorithm.

    Source: https://en.wikipedia.org/wiki/A*_search_algorithm
    """

    class Spot:
        """
        Represents a spot (occupied or not) on the grid.
        """

        def __init__(self, x: int, y: int, occupied: bool):
            """
            Represents a spot (occupied or not) on the grid.
            :param x: Vertical position
            :param y: Horizontal position
            :param occupied: True for walls or any other obstacles
            """
            self.x = x
            self.y = y

            # Initialise f and g to extreme values ("nearly" infinity)
            self.f = 10_000_000
            self.g = 10_000_000
            self.h = 0

            self.neighbours = []  # Neighbours of this spot - cells around it

            self.previous = None  # This will be used for the path
            self.occupied = occupied

        def add_neighbour(self, neighbour):
            self.neighbours.append(neighbour)

        def __str__(self):
            return f"({self.x}, {self.y})"

    def __init__(self, grid: numpy.ndarray):
        """
        A Star path-finding algorithm.
        :param grid: Binary occupancy grid represented as a 2D numpy array.
        """
        self.spots: Dict = {}
        self._parse_spots(grid)  # Translate the grid into spots

    def _parse_spots(self, grid):
        rows = grid.shape[0]
        columns = grid.shape[1]

        for x in range(0, rows):
            for y in range(0, columns):
                new_spot = AStar.Spot(x, y, grid[x][y] > 0)
                self.spots[(x, y)] = new_spot

                # Add neighbours
                if x > 0:
                    self._create_neighbour_pair(new_spot, self.spots[(x - 1, y)])
                if y > 0:
                    self._create_neighbour_pair(new_spot, self.spots[(x, y - 1)])
                if x > 0 and y > 0:
                    self._create_neighbour_pair(new_spot, self.spots[(x - 1, y - 1)])
                if x > 0 and y < columns - 1:
                    self._create_neighbour_pair(new_spot, self.spots[(x - 1, y + 1)])

    @staticmethod
    def _create_neighbour_pair(spot_a: Spot, spot_b: Spot):
        spot_a.add_neighbour(spot_b)
        spot_b.add_neighbour(spot_a)

    def find_path(self, start: (int, int), goal: (int, int)) -> Optional[List]:
        """
        Finds the shortest path between the two given points
        :param start: The starting position
        :param goal: The goal position
        :return: List of (x, y) coordinates from start to the goal, or None if path not found.
        """
        start_spot = self.spots[start]
        goal_spot = self.spots[goal]

        if start_spot.occupied or goal_spot.occupied:
            raise AStarError("Start and goal spots cannot be occupied")

        open_set = set()
        open_set.add(start_spot)

        start_spot.g = 0
        start_spot.f = 0

        while open_set:
            current_spot = min(open_set, key=lambda spot: spot.f)
            if current_spot == goal_spot:
                return self._reconstruct_path(current_spot)

            open_set.remove(current_spot)
            for neighbour in current_spot.neighbours:
                if neighbour.occupied:
                    continue

                tentative_g = current_spot.g + self._distance_between(current_spot, neighbour)
                if tentative_g < neighbour.g:
                    neighbour.g = tentative_g
                    neighbour.h = self._distance_between(neighbour, goal_spot)
                    neighbour.f = neighbour.g + neighbour.h
                    neighbour.previous = current_spot

                    if neighbour not in open_set:
                        open_set.add(neighbour)

        return None

    @staticmethod
    def _reconstruct_path(current) -> List:
        path = []
        while current is not None:
            path.append(current)
            current = current.previous

        # Don't need to expose the Spot, return as (x, y)
        # But we need to reverse it first, since we're working from the end.
        return list(map(lambda spot: (spot.x, spot.y), reversed(path)))

    @staticmethod
    def _distance_between(spot_a: Spot, spot_b: Spot) -> int:
        """
        Chebyshev distance between the two points.
        :param spot_a:
        :param spot_b:
        :return: Chebyshev distance
        """
        return max(abs(spot_b.x - spot_a.x), abs(spot_b.y - spot_a.y))


class AStarError(Exception):
    def __init__(self, message):
        super().__init__(message)
