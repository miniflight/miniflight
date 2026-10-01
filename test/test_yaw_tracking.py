import math
import unittest

from target.aigp.yaw_tracking import YawRateFeedback


class YawFeedbackTest(unittest.TestCase):
    def test_new_reference_does_not_charge_error_before_it_was_applied(self):
        feedback = YawRateFeedback()
        self.assertEqual(feedback.update(0, 0, .02), 0)
        self.assertEqual(feedback.update(.5, 0, .02), .5)
        for _ in range(8):
            self.assertEqual(feedback.update(.5, .45, .02), .5)
        self.assertGreater(feedback.update(.5, .45, .02), .5)

    def test_removes_unknown_steady_gain_error_in_either_direction(self):
        for gain in (.8, .9, 1.05):
            for requested in (-.5, .5):
                with self.subTest(gain=gain, requested=requested):
                    feedback = YawRateFeedback()
                    measured = 0.0
                    for _ in range(250):
                        command = feedback.update(requested, measured, .02)
                        measured += (gain * command - measured) * .02 / .06
                    self.assertAlmostEqual(measured, requested, delta=.001)

    def test_neutral_and_mode_reset_clear_the_previous_correction(self):
        feedback = YawRateFeedback()
        for _ in range(30):
            feedback.update(.25, .22, .02)
        self.assertGreater(feedback.correction, 0)
        self.assertEqual(feedback.update(0, .2, .02), 0)
        self.assertEqual(feedback.correction, 0)
        feedback.update(.25, .22, .02)
        feedback.reset()
        self.assertEqual(feedback.correction, 0)

    def test_correction_and_output_are_bounded_and_unwind_at_saturation(self):
        feedback = YawRateFeedback()
        for _ in range(200):
            self.assertLessEqual(feedback.update(.9, 0, .02), 1)
        self.assertLessEqual(feedback.correction, feedback.max_correction)
        previous = feedback.correction
        feedback.update(.9, 1, .02)
        self.assertLess(feedback.correction, previous)
        for _ in range(200):
            self.assertGreaterEqual(feedback.update(-.9, 0, .02), -1)

    def test_discontinuous_time_does_not_integrate_a_gap(self):
        for dt in (0, -.01, .2):
            feedback = YawRateFeedback()
            feedback.update(.5, 0, .02)
            self.assertEqual(feedback.update(.5, 0, dt), .5)
            self.assertEqual(feedback.correction, 0)

    def test_nonfinite_inputs_and_invalid_parameters_are_rejected(self):
        for values in ((math.nan, 0, .02), (0, math.inf, .02), (0, 0, math.nan)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                YawRateFeedback().update(*values)
        for values in (dict(gain=0), dict(max_correction=-1), dict(max_rate=math.inf)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                YawRateFeedback(**values)
