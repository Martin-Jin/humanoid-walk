import math
import unittest
from dataclasses import replace

from humanoid_walk.config import load_config
from humanoid_walk.controllers.gait import GaitController
from humanoid_walk.controllers.joint_state import JointName


def walking_gait(config=None) -> GaitController:
    """A controller past the start-up sequence, at full walk, with the clock at 0."""
    gait = GaitController(config or load_config())
    gait.elapsed_s = gait.stand_s + gait.crouch_s + gait.ramp_s + 1.0
    return gait


class GaitTests(unittest.TestCase):
    def test_ik_reaches_the_foot_target(self):
        """Forward kinematics of the IK result lands on the requested foot position."""
        gait = GaitController(load_config())
        for x, y in [(0.15, 0.80), (-0.1, 0.85), (0.0, 0.88)]:
            hip, knee = gait._leg_ik(x, y)
            thigh_dir = -hip  # forward angle from straight down
            shin_dir = -hip - knee  # a positive knee swings the shin backward
            fx = gait.thigh * math.sin(thigh_dir) + gait.shin * math.sin(shin_dir)
            fy = gait.thigh * math.cos(thigh_dir) + gait.shin * math.cos(shin_dir)
            self.assertAlmostEqual(fx, x, places=6)
            self.assertAlmostEqual(fy, y, places=6)
            self.assertGreaterEqual(knee, 0.0)

    def test_foot_target_stance_start(self):
        gait = walking_gait()
        gait.crouch, gait.walk = 1.0, 1.0
        left = gait.legs[0]
        x, y = gait._foot_target(left, True, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(x, gait.gait.step_length_m / 2)
        self.assertAlmostEqual(y, gait.gait.hip_height_m)

    def test_act_covers_every_joint(self):
        targets = GaitController(load_config()).act({})
        self.assertEqual(set(targets), set(JointName))

    def test_ankle_within_limits(self):
        gait = GaitController(load_config())
        lo, hi = gait.ankle_limits
        for _ in range(600):
            ankle = gait.act({})[JointName.ANKLE_L]
            self.assertTrue(lo <= ankle <= hi)
            gait.step_clock(0.01)

    def test_clock_waits_for_the_startup_sequence(self):
        gait = GaitController(load_config())
        gait.step_clock(gait.stand_s + gait.crouch_s - 0.1)
        self.assertEqual(gait.time_s, 0.0)
        gait.step_clock(0.5)
        gait.step_clock(0.5)
        self.assertGreater(gait.time_s, 0.0)

    def test_right_leg_is_half_a_cycle_behind(self):
        gait = walking_gait()
        left = gait.act({})[JointName.HIP_L]
        gait.time_s = gait.gait.period_s / 2
        right_later = gait.act({})[JointName.HIP_R]
        self.assertAlmostEqual(left, right_later)

    def test_reset_restarts_clock_and_startup(self):
        gait = walking_gait()
        gait.step_clock(0.7)
        gait.reset()
        self.assertEqual(gait.time_s, 0.0)
        self.assertEqual(gait.elapsed_s, 0.0)

    def test_clock_holds_for_contact_then_carries_on(self):
        """A finished swing with no contact holds the clock, but only up to hold_max_s."""
        gait = walking_gait()
        gait.time_s = 0.996 * gait.gait.period_s  # left leg at the very end of swing
        gait.set_contacts(False, True)
        gait.step_clock(0.05)
        self.assertAlmostEqual(gait.time_s, 0.996 * gait.gait.period_s)
        for _ in range(30):
            gait.step_clock(0.01)
        self.assertGreater(gait.time_s, 0.996 * gait.gait.period_s)

    def test_gate_pauses_when_capture_point_leaves_the_feet(self):
        config = load_config()
        gate = {**config.gait.balance_gate, "enabled": 1.0}
        gait = walking_gait(replace(config, gait=replace(config.gait, balance_gate=gate)))
        gait.time_s = 0.05 * gait.gait.period_s  # both feet in stance (double support)
        gait.set_com({JointName.ANKLE_L: 0.0, JointName.ANKLE_R: 0.0}, 2.0, 0.9)
        gait._update_gate()
        self.assertTrue(gait.paused)


if __name__ == "__main__":
    unittest.main()
