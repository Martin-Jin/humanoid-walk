import math
import unittest

from humanoid_walk.config import load_config
from humanoid_walk.controllers.gait import GaitController, foot_target, leg_ik
from humanoid_walk.controllers.joint_state import JointName


class GaitTests(unittest.TestCase):
    def test_ik_matches_docs_table(self):
        hip, knee = leg_ik(0.15, 0.80, 0.45, 0.45)
        self.assertAlmostEqual(hip, -0.626, places=3)
        self.assertAlmostEqual(knee, 0.882, places=3)

    def test_foot_target_stance_start(self):
        x, y = foot_target(0.0, 0.30, 0.08, 0.80, 0.6)
        self.assertAlmostEqual(x, 0.15)
        self.assertAlmostEqual(y, 0.80)

    def test_act_covers_every_joint(self):
        targets = GaitController(load_config()).act({})
        self.assertEqual(set(targets), set(JointName))

    def test_ankle_within_limits(self):
        gait = GaitController(load_config())
        lo, hi = gait.ankle_limits
        for _ in range(100):
            ankle = gait.act({})[JointName.ANKLE_L]
            self.assertTrue(lo <= ankle <= hi)
            gait.step_clock(0.01)

    def test_right_leg_is_half_a_cycle_behind(self):
        gait = GaitController(load_config())
        left = gait.act({})[JointName.HIP_L]
        gait.step_clock(gait.gait.period_s / 2)
        right_later = gait.act({})[JointName.HIP_R]
        self.assertAlmostEqual(left, right_later)

    def test_reset_restarts_clock(self):
        gait = GaitController(load_config())
        gait.step_clock(0.7)
        gait.reset()
        self.assertEqual(gait.time_s, 0.0)


if __name__ == "__main__":
    unittest.main()
