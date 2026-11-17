import unittest

from humanoid_walk.config import load_config
from humanoid_walk.controllers.gait import GaitController
from humanoid_walk.controllers.joint_control import make_body_controller
from humanoid_walk.controllers.joint_state import JointName
from humanoid_walk.sim.physics import Simulation


def simulate_walk(seconds: float, push_mps: float = 0.0, push_at_s: float = 10.0):
    """Run the viewer's control loop headless. Returns (fell_at_s or None, distance in m)."""
    config = load_config()
    sim = Simulation(config)
    sim.setup()
    joint_cfg = {j.name: j for j in config.humanoid.joint_configs}
    body = make_body_controller(
        config, joint_cfg, config.joint_controller_names, config.controllers
    )
    gait = GaitController(config)
    hz, ppm = config.world.sim_hz, sim.ppm
    start_x = sim.torso.position.x
    for frame in range(int(seconds * hz)):
        t = frame / hz
        if push_mps and abs(t - push_at_s) < 0.5 / hz:
            sim.torso.velocity = sim.torso.velocity + (push_mps * ppm, 0)
        gait.set_torso(sim.torso.angle, sim.torso.angular_velocity)
        dx_l, vx = sim.com_vs_ankle(JointName.ANKLE_L)
        dx_r, _ = sim.com_vs_ankle(JointName.ANKLE_R)
        height = max(
            sim.com_height_above_ankle(JointName.ANKLE_L),
            sim.com_height_above_ankle(JointName.ANKLE_R),
        )
        gait.set_com({JointName.ANKLE_L: dx_l, JointName.ANKLE_R: dx_r}, vx, height)
        gait.set_contacts(*sim.feet_on_ground())
        targets = gait.act(sim.joint_states())
        sim.control_step(body, targets)
        gait.step_clock(1.0 / hz)
        torso_height_m = (config.rendering.floor_y_px - sim.torso.position.y) / ppm
        if abs(sim.torso.angle) > 0.7 or torso_height_m < 1.0:
            return t, (sim.torso.position.x - start_x) / ppm
    return None, (sim.torso.position.x - start_x) / ppm


class WalkTests(unittest.TestCase):
    """The gait is chaotic, so these catch a retune or refactor that breaks walking."""

    def test_walks_without_falling(self):
        fell_at, distance = simulate_walk(25.0)
        self.assertIsNone(fell_at, f"fell at {fell_at} s")
        self.assertGreater(distance, 3.0)

    def test_survives_a_push_forward_and_back(self):
        for push in (0.3, -0.3):
            fell_at, _ = simulate_walk(18.0, push_mps=push, push_at_s=10.0)
            self.assertIsNone(fell_at, f"push {push} m/s: fell at {fell_at} s")


if __name__ == "__main__":
    unittest.main()
