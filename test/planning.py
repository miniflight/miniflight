import unittest
import json
from pathlib import Path

import numpy as np

from target.aigp.experiments.plan_motion import plan
from target.aigp.experiments.plan_race import crossing_constraints, force_for_motion, integrate_motion, motion_constraints, plan as plan_race


class MotionPlanningTest(unittest.TestCase):
    def test_search_selects_a_shorter_feasible_crossing_time(self):
        model = json.loads((Path(__file__).parent / "fixtures/vq1_r1_model.json").read_text())
        reference = dict(states=np.zeros((21, 3, 3)), acceleration_rates=np.zeros((20, 3)),
                         gates=np.array([20]), points=np.array([[0, 0, 0], [-1, 0, 0]]), dt=.2)
        result = plan_race(reference, model)
        self.assertLess(result["times"][-1], 3)
        self.assertAlmostEqual(result["states"][-1, 0, 0], -1, places=4)
        self.assertGreaterEqual(result["minimum_margin"], -1e-5)
        result["points"][1, 1] += 1  # A prior chosen crossing must not move the physical gate.
        replanned = plan_race(result, model)
        self.assertLessEqual(abs(replanned["states"][-1, 0, 1]), model["clearance_halfspan_m"][0] + 1e-5)

    def test_crossings_clear_the_opening_and_follow_its_permitted_direction(self):
        model = json.loads((Path(__file__).parent / "fixtures/vq1_r1_model.json").read_text())
        with np.load(Path(__file__).parent / "fixtures/vq1_r1_lift_30s.npz") as source:
            states, gates, centers = source["states"], source["gates"], source["points"][1:]
            planes, margins = crossing_constraints(states, gates, centers, model)
            np.testing.assert_allclose(planes, 0, atol=1e-8)
            self.assertGreaterEqual(margins.min(), 0)
            states[gates[0], 0, 1] += 1
            self.assertLess(crossing_constraints(states, gates, centers, model)[1].min(), 0)

    def test_hover_is_feasible_but_fast_motion_exceeds_the_force_budget(self):
        model = json.loads((Path(__file__).parent / "fixtures/vq1_r1_model.json").read_text())
        initial, jerk, intervals = np.zeros((3, 3)), np.zeros((2, 3)), np.array([.5, .5])
        self.assertGreaterEqual(motion_constraints(initial, jerk, intervals, model).min(), 0)
        initial[1, 0] = 10
        self.assertLess(motion_constraints(initial, jerk, intervals, model).min(), 0)

    def test_required_thrust_balances_gravity_and_directional_resistance(self):
        velocity = np.array([[0, 0, 0], [3, 4, -2]])
        force = force_for_motion(velocity, np.zeros((2, 3)), [.05, .05, .2])
        np.testing.assert_allclose(force, [[0, 0, -9.81], [.75, 1, -10.61]])

    def test_variable_intervals_integrate_the_same_jerk_exactly(self):
        initial = np.array([[0, 0, 0], [1, 0, 0], [0, 0, 0]])
        states = integrate_motion(initial, np.array([[1, 0, 0], [1, 0, 0]]), [.5, 1.5])
        np.testing.assert_allclose(states[-1], [[10/3, 0, 0], [3, 0, 0], [2, 0, 0]])

    def test_lift_and_gate_constraints_at_each_pace(self):
        with np.load(Path(__file__).parent / "fixtures/vq1_r1_35s.npz") as source:
            for duration in (35, 32, 30):
                result = plan(source, duration)
                states, jerk, dt = result["states"], result["acceleration_rates"], result["dt"]
                np.testing.assert_array_equal(states[0], [[0, 0, 0], [0, 0, 0], [0, 0, -2]])
                np.testing.assert_allclose(states[result["gates"], 0], result["points"][1:], atol=1e-8)
                np.testing.assert_allclose(states[1:, 0], states[:-1, 0] + states[:-1, 1]*dt + states[:-1, 2]*dt**2/2 + jerk*dt**3/6, atol=1e-8)
                np.testing.assert_allclose(states[1:, 1], states[:-1, 1] + states[:-1, 2]*dt + jerk*dt**2/2, atol=1e-8)
                np.testing.assert_allclose(states[1:, 2], states[:-1, 2] + jerk*dt, atol=1e-8)
                self.assertLessEqual(abs(states[:, 2]).max(), 2.1 + 1e-8)
                self.assertLessEqual(abs(jerk).max(), 4 + 1e-8)
