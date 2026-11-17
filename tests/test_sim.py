import unittest
from dataclasses import replace

from humanoid_walk.config import load_config
from humanoid_walk.sim.physics import Simulation


class ContactTests(unittest.TestCase):
    def make_sim(self, hold_s: float) -> Simulation:
        config = load_config()
        config = replace(config, world=replace(config.world, contact_hold_s=hold_s))
        sim = Simulation(config)
        sim.setup()
        return sim

    def test_foot_stays_down_for_the_hold_time_after_contact(self):
        sim = self.make_sim(0.03)
        sim.time_s = 1.0
        sim._last_contact[sim.foot_l] = 1.0 - 0.02  # touched 20 ms ago
        sim._last_contact[sim.foot_r] = 1.0 - 0.05  # touched 50 ms ago
        self.assertEqual(sim.feet_on_ground(), (True, False))

    def test_foot_that_never_touched_is_not_down(self):
        self.assertEqual(self.make_sim(0.03).feet_on_ground(), (False, False))

    def test_reset_forgets_old_contacts(self):
        sim = self.make_sim(0.03)
        sim._last_contact[sim.foot_l] = sim.time_s
        sim.reset()
        self.assertEqual(sim.feet_on_ground(), (False, False))

    def test_humanoid_lands_and_reports_contact(self):
        sim = self.make_sim(0.03)
        for _ in range(120):
            sim.step()
        self.assertEqual(sim.feet_on_ground(), (True, True))


if __name__ == "__main__":
    unittest.main()
