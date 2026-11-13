""" "Utility functions for debugging the simulation and rendering."""

from humanoid_walk.controllers.joint_state import JointName


def print_walk_state(app):
    """Print relevant info about walk state such as torques for relevant joints."""
    app.frame += 1
    if app.frame % 20 == 0:
        states = app.sim.joint_states()
        torques = app.body.compute_torques(app.targets, states)
        for name in (JointName.HIP_L, JointName.KNEE_L, JointName.ANKLE_L):
            print(
                f"{name.value:8} target {app.targets[name]:+.2f} "
                f"actual {states[name].current:+.2f} torque {torques[name]:+7.1f}"
            )
        print("feet on ground:", app.sim.feet_on_ground(), "\n")
        ppm = app.sim.ppm
        print(f"pitch={app.gait.torso_pitch:.2f} rate={app.gait.torso_rate:.2f}")
        print(
            f"t {app.gait.time_s:.2f}  "
            f"torso vx {app.sim.torso.velocity.x / ppm:+.2f} m/s  "
            f"foot_l vx {app.sim.foot_l.velocity.x / ppm:+.2f}  "
            f"foot_r vx {app.sim.foot_r.velocity.x / ppm:+.2f}"
        )
        print(app.sim.com_vs_ankle(JointName.ANKLE_L))
