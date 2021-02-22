import unittest

from PIL import Image

from nav.src import image_parser


class ImageParserTest(unittest.TestCase):
    def test_image_parser_data(self):
        """
        Test if the array that is returned from the image parser is actually what we expect.
        """
        image = Image.open('../test_resources/simple_bw.bmp')
        expected = [[1] * 10,
                    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                    [1, 0, 0, 0, 0, 0, 0, 1, 1, 1],
                    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                    [1, 1, 1, 1, 1, 0, 0, 0, 0, 1],
                    [1, 0, 0, 0, 1, 0, 0, 0, 0, 1],
                    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                    [1] * 10]

        actual = image_parser.parse_image(image).tolist()
        self.assertListEqual(expected, actual)
