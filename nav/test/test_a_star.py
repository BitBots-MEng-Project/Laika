import unittest

from PIL import Image

import nav.src.a_star as path_finder
import nav.src.image_parser as image_parser


class AStarTest(unittest.TestCase):
    def setUp(self) -> None:
        grid = image_parser.parse_image(Image.open('../test_resources/simple_bw.bmp'))
        self.a_star = path_finder.AStar(grid)

    def test_single_spot(self):
        self.assertTrue(self.a_star.spots[(6, 4)].occupied)  # Check for one spot

        # Make sure its left and right neighbours are good as well.
        self.assertFalse(self.a_star.spots[(6, 3)].occupied)
        self.assertFalse(self.a_star.spots[(6, 5)].occupied)

    def test_neighbours(self):
        # By picking the spot/pixel at the position (1, 1) (0-indexing) we will check
        # for its surrounding neighbours
        spot = self.a_star.spots[(1, 1)]
        expected_neighbours = [
            self.a_star.spots[(0, 0)],
            self.a_star.spots[(0, 1)],
            self.a_star.spots[(0, 2)],
            self.a_star.spots[(1, 0)],
            self.a_star.spots[(1, 2)],
            self.a_star.spots[(2, 0)],
            self.a_star.spots[(2, 1)],
            self.a_star.spots[(2, 2)],
        ]
        actual_neighbours = spot.neighbours
        self.assertCountEqual(expected_neighbours, actual_neighbours)

    def test_raises_exception_when_start_occupied(self):
        self.assertRaises(path_finder.AStarError, self.a_star.find_path, (0, 0), (1, 1))

    def test_raises_exception_when_goal_occupied(self):
        self.assertRaises(path_finder.AStarError, self.a_star.find_path, (1, 1), (9, 9))

    def test_valid_path_is_not_none(self):
        path = self.a_star.find_path((6, 2), (2, 8))
        self.assertIsNotNone(path)

    def test_valid_path(self):
        start = (6, 2)
        goal = (2, 8)

        expected = [
            (6, 2),
            (7, 3),
            (7, 4),
            (6, 5),
            (5, 5),
            (4, 6),
            (3, 6),
            (2, 7),
            (2, 8),
        ]
        actual = self.a_star.find_path(start, goal)

        self.assertListEqual(expected, actual)
