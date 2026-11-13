""" "Utility functions for debugging the simulation and rendering."""

from humanoid_walk.controllers.joint_state import JointName


def print_walk_state(sim):
    """Print relevant info about walk state such as torques for relevant joints."""
    sim.frame += 1
    if sim.frame % 20 == 0:
        states = sim.sim.joint_states()
        torques = sim.body.compute_torques(sim.targets, states)
        for name in (JointName.HIP_L, JointName.KNEE_L, JointName.ANKLE_L):
            print(
                f"{name.value:8} target {sim.targets[name]:+.2f} "
                f"actual {states[name].current:+.2f} torque {torques[name]:+7.1f}"
            )
        print("feet on ground:", sim.sim.feet_on_ground(), "\n")
        ppm = sim.sim.ppm
        print(f"pitch={sim.gait.torso_pitch:.2f} rate={sim.gait.torso_rate:.2f}")
        print(
            f"t {sim.gait.time_s:.2f}  "
            f"torso vx {sim.sim.torso.velocity.x / ppm:+.2f} m/s  "
            f"foot_l vx {sim.sim.foot_l.velocity.x / ppm:+.2f}  "
            f"foot_r vx {sim.sim.foot_r.velocity.x / ppm:+.2f}"
        )
