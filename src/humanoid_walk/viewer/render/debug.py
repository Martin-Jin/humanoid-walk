"""Utility functions for debugging the simulation and rendering."""

from humanoid_walk.controllers.joint_state import JointName

PIN_TOL = 0.02  # rad, a target this close to a joint limit is flagged as pinned


def _limits(app):
    cfg = {j.name: j for j in app.config.humanoid.joint_configs}
    return {name: cfg[name.value.split("_")[0]].limits for name in JointName}


def print_walk_state(app):
    """Print gait, balance and joint info every 20 frames."""
    app.frame += 1
    if app.frame % 20:
        return

    g, sim = app.gait, app.sim
    ppm = sim.ppm
    states = sim.joint_states()
    torques = app.body.compute_torques(app.targets, states)
    limits = _limits(app)
    contacts = dict(zip((JointName.ANKLE_L, JointName.ANKLE_R), sim.feet_on_ground()))

    print(
        f"elapsed {g.elapsed_s:5.2f}  clock {g.time_s:5.2f}  "
        f"crouch {g.crouch:.2f} walk {g.walk:.2f}  "
        f"gate {g.gate_scale:.2f}{' PAUSED' if g.paused else ''}"
    )
    for leg in g.legs:
        in_stance, s, standing = g._leg_state(leg)
        phase = "stand" if standing else ("stance" if in_stance else "swing ")
        side = leg.hip.value[-1].upper()
        print(
            f" {side} {phase} s={s:.2f}  "
            f"ground={contacts[leg.ankle]!s:5}  "
            f"com_dx={g.com_dx[leg.ankle]:+.3f} m"
        )
        for joint in (leg.hip, leg.knee, leg.ankle):
            lo, hi = limits[joint]
            tgt = app.targets[joint]
            pin = " PINNED" if tgt <= lo + PIN_TOL or tgt >= hi - PIN_TOL else ""
            print(
                f"    {joint.value:8} target {tgt:+.2f} "
                f"actual {states[joint].current:+.2f} "
                f"torque {torques[joint]:+7.1f}{pin}"
            )
    print(
        f" torso pitch {g.torso_pitch:+.2f} rate {g.torso_rate:+.2f}  "
        f"vx {sim.torso.velocity.x / ppm:+.2f} m/s  "
        f"com_vx {g.com_vx:+.2f} m/s  com_h {g.com_height:.2f} m"
    )
    print()
