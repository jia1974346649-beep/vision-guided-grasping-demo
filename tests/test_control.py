import unittest

import numpy as np

from grasp_demo.control import PID, dls_step


class PIDTests(unittest.TestCase):
    def test_numeric_terms_and_first_sample_has_no_derivative_kick(self):
        controller = PID(2, 0.5, 0.1, 0.1, 10)
        self.assertAlmostEqual(controller.compute(1), 2.05)
        self.assertAlmostEqual(controller.compute(0.5), 0.575)

    def test_output_saturates_per_axis(self):
        controller = PID(2, 1, 0, 0.1, 0.5)
        np.testing.assert_allclose(controller.compute([10, -10, 0]), [0.5, -0.5, 0])
        for _ in range(100):
            np.testing.assert_allclose(controller.compute([10, -10, 0]), [0.5, -0.5, 0])
        np.testing.assert_allclose(controller.compute([-0.1, 0.1, 0]), [-0.21, 0.21, 0])

    def test_integrator_can_unwind_after_saturation(self):
        controller = PID(0, 1, 0, 0.1, 1)
        for _ in range(100):
            self.assertLessEqual(abs(controller.compute(1)), 1)
        self.assertAlmostEqual(controller.compute(-0.2), 0.98)
        for _ in range(49):
            result = controller.compute(-0.2)
        self.assertAlmostEqual(result, 0, places=12)

    def test_reset_clears_history_and_shape_binding(self):
        controller = PID(1, 1, 1, 0.1, 100)
        controller.compute([1, 2])
        with self.assertRaises(ValueError):
            controller.compute(1)
        controller.reset()
        self.assertAlmostEqual(controller.compute(1), 1.1)

    def test_does_not_alias_input_or_returned_state(self):
        controller = PID(1, 0, 0, 0.1, 10)
        error = np.array([1.0, 2.0])
        output = controller.compute(error)
        np.testing.assert_array_equal(error, [1, 2])
        error[:] = 9
        output[:] = 8
        np.testing.assert_array_equal(controller.compute([0, 0]), [0, 0])

    def test_invalid_parameters(self):
        for index, values in enumerate(
            [(float("nan"), 0, 0, 0.1, 1), (-1, 0, 0, 0.1, 1),
             (1, float("inf"), 0, 0.1, 1), (1, 0, -1, 0.1, 1),
             (1, 0, 0, 0, 1), (1, 0, 0, -0.1, 1),
             (1, 0, 0, 0.1, 0), (1, 0, 0, 0.1, float("inf")),
             (True, 0, 0, 0.1, 1)]
        ):
            with self.subTest(index=index), self.assertRaises(ValueError):
                PID(*values)

    def test_invalid_error_does_not_poison_history(self):
        controller = PID(1, 1, 0, 0.1, 10)
        controller.compute(1)
        for value in (float("nan"), float("inf"), [], "1", True, complex(1, 2)):
            with self.subTest(value=value), self.assertRaises(ValueError):
                controller.compute(value)
        self.assertAlmostEqual(controller.compute(1), 1.2)


class DampedLeastSquaresTests(unittest.TestCase):
    def test_identity_has_known_damped_solution(self):
        expected = np.array([0.2, -0.1, 0.4]) / (1 + 0.1 ** 2)
        np.testing.assert_allclose(dls_step(np.eye(3), [0.2, -0.1, 0.4], 0.1, 1), expected)

    def test_rectangular_solution_reduces_task_error(self):
        matrix = np.array([[1, 0, 1], [0, 1, 1]], dtype=float)
        error = np.array([0.02, -0.01])
        step = dls_step(matrix, error)
        self.assertEqual(step.shape, (3,))
        self.assertLess(np.linalg.norm(error - matrix @ step), np.linalg.norm(error))

    def test_rank_deficient_and_zero_jacobians_remain_finite(self):
        for matrix in (np.zeros((3, 7)), np.ones((3, 7)), np.array([[1, 2], [2, 4]])):
            step = dls_step(matrix, np.ones(matrix.shape[0]))
            self.assertTrue(np.all(np.isfinite(step)))
            self.assertLessEqual(float(np.max(np.abs(step))), 0.08)

    def test_clipping_is_per_joint(self):
        np.testing.assert_allclose(dls_step(np.eye(2), [10, -10], max_step=0.04), [0.04, -0.04])

    def test_inputs_are_unchanged(self):
        matrix = np.eye(2)
        error = np.array([1.0, 2.0])
        dls_step(matrix, error)
        np.testing.assert_array_equal(matrix, np.eye(2))
        np.testing.assert_array_equal(error, [1, 2])

    def test_invalid_shapes_and_values(self):
        for matrix, error in (([], []), ([1, 2], [1]), (np.zeros((0, 2)), []),
                              (np.eye(2), [1]), (np.eye(2), [[1], [2]]),
                              ([[float("nan")]], [1]), ([[1]], [float("inf")]),
                              ([[1 + 1j]], [1]), ([["1"]], [1])):
            with self.subTest(matrix=matrix, error=error), self.assertRaises(ValueError):
                dls_step(matrix, error)

    def test_invalid_damping_or_increment_limit(self):
        for keyword in ("damping", "max_step"):
            for value in (0, -1, float("nan"), float("inf"), True):
                with self.subTest(keyword=keyword, value=value), self.assertRaises(ValueError):
                    dls_step([[1]], [1], **{keyword: value})


if __name__ == "__main__":
    unittest.main()
