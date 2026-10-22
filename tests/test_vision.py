import unittest

import numpy as np

from grasp_demo.vision import detect_red, red_mask


def blank(height=10, width=12):
    return np.zeros((height, width, 3), dtype=np.uint8), np.full((height, width), 0.5)


class RedLocalizationTests(unittest.TestCase):
    def test_no_target_returns_none(self):
        rgb, depth = blank()
        self.assertIsNone(detect_red(rgb, depth, np.eye(4), np.eye(4)))

    def test_small_noise_below_area_threshold_is_ignored(self):
        rgb, depth = blank()
        rgb[2:4, 3:5] = [255, 0, 0]
        self.assertIsNone(detect_red(rgb, depth, np.eye(4), np.eye(4), min_area=5))

    def test_hue_wrap_red_and_magenta_boundary(self):
        rgb = np.array([[[255, 0, 0], [255, 0, 40], [255, 40, 0],
                         [255, 0, 255], [0, 255, 0], [0, 0, 255],
                         [100, 100, 100], [10, 0, 0], [255, 220, 220]]], dtype=np.uint8)
        np.testing.assert_array_equal(red_mask(rgb), [[True, True, True, False, False, False, False, False, False]])

    def test_largest_component_and_pixel_center_identity_projection(self):
        rgb, depth = blank()
        rgb[1:3, 1:3] = [255, 0, 0]
        rgb[5:9, 6:11] = [255, 0, 20]
        result = detect_red(rgb, depth, np.eye(4), np.eye(4), min_area=4)
        self.assertEqual(result["area"], 20)
        np.testing.assert_allclose(result["pixel"], [8, 6.5])
        np.testing.assert_allclose(result["surface_point"], [2 * 8.5 / 12 - 1, 1 - 2 * 7 / 10, 0])

    def test_diagonal_noise_does_not_form_connected_component(self):
        rgb, depth = blank()
        for index in range(6):
            rgb[index, index] = [255, 0, 0]
        self.assertIsNone(detect_red(rgb, depth, np.eye(4), np.eye(4), min_area=2))

    def test_camera_translation_and_flat_column_major_matrices(self):
        rgb, depth = blank()
        rgb[3:7, 4:8] = [255, 0, 0]
        view = np.eye(4)
        view[:3, 3] = [-2, 3, -4]
        result = detect_red(rgb, depth, view.flatten(order="F"), np.eye(4).flatten(order="F"))
        np.testing.assert_allclose(result["pixel"], [5.5, 4.5])
        np.testing.assert_allclose(result["surface_point"], [2, -3, 4])

    def test_known_perspective_calibration_recovers_world_point(self):
        # 90-degree perspective, aspect 1, near=1, far=10. A center pixel
        # at camera z=-4 has OpenGL depth 5/6; no renderer is used here.
        rgb, depth = blank(9, 9)
        rgb[2:7, 2:7] = [255, 0, 0]
        depth[:] = 5 / 6
        projection = np.array([[1, 0, 0, 0], [0, 1, 0, 0],
                               [0, 0, -11 / 9, -20 / 9], [0, 0, -1, 0]])
        view = np.eye(4)
        view[:3, 3] = [-1, 2, -3]
        result = detect_red(rgb, depth, view.flatten(order="F"), projection.flatten(order="F"))
        np.testing.assert_allclose(result["surface_point"], [1, -2, -1], atol=1e-12)

    def test_only_valid_component_depth_contributes(self):
        rgb, depth = blank()
        rgb[3:7, 4:8] = [255, 0, 0]
        depth[3:7, 4:8] = 1
        depth[4, 5] = 0.25
        result = detect_red(rgb, depth, np.eye(4), np.eye(4))
        self.assertEqual(result["area"], 16)
        np.testing.assert_allclose(result["pixel"], [5, 4])
        np.testing.assert_allclose(result["surface_point"], [2 * 5.5 / 12 - 1, 1 - 2 * 4.5 / 10, -0.5])

    def test_component_with_only_background_or_near_plane_has_no_target(self):
        rgb, depth = blank()
        rgb[3:7, 4:8] = [255, 0, 0]
        for value in (0, 1):
            depth[:] = value
            self.assertIsNone(detect_red(rgb, depth, np.eye(4), np.eye(4)))

    def test_inputs_not_mutated(self):
        rgb, depth = blank()
        rgb[3:7, 4:8] = [255, 0, 0]
        before_rgb, before_depth = rgb.copy(), depth.copy()
        view, projection = np.eye(4), np.eye(4)
        detect_red(rgb, depth, view, projection)
        np.testing.assert_array_equal(rgb, before_rgb)
        np.testing.assert_array_equal(depth, before_depth)
        np.testing.assert_array_equal(view, np.eye(4))
        np.testing.assert_array_equal(projection, np.eye(4))

    def test_invalid_rgb(self):
        for rgb in (np.zeros((10, 12)), np.zeros((10, 12, 4), dtype=np.uint8),
                    np.zeros((0, 12, 3), dtype=np.uint8), np.zeros((10, 12, 3))):
            with self.subTest(shape=rgb.shape), self.assertRaises(ValueError):
                red_mask(rgb)

    def test_invalid_depth(self):
        rgb, depth = blank()
        for bad in (depth[:-1], depth + 1, depth - 1, np.full_like(depth, np.nan),
                    np.full_like(depth, np.inf), depth.astype(complex)):
            with self.subTest(shape=bad.shape), self.assertRaises(ValueError):
                detect_red(rgb, bad, np.eye(4), np.eye(4))

    def test_invalid_camera_matrices(self):
        rgb, depth = blank()
        for bad in ([1, 2], np.zeros((4, 4)), np.full((4, 4), np.nan), np.eye(4, dtype=complex)):
            with self.subTest(matrix=bad), self.assertRaises(ValueError):
                detect_red(rgb, depth, bad, np.eye(4))
            with self.subTest(matrix=bad), self.assertRaises(ValueError):
                detect_red(rgb, depth, np.eye(4), bad)

    def test_invalid_minimum_area(self):
        rgb, depth = blank()
        for value in (0, -1, 2.5, True, "15"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                detect_red(rgb, depth, np.eye(4), np.eye(4), min_area=value)


if __name__ == "__main__":
    unittest.main()
