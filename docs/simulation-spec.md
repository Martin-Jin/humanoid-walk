# Simulation specification

This document owns the body, action set, observation layout, reward definition and
timing. Update it in the same change as any of them. Values live in `configs/` (`sim.yaml`,
`humanoid.yaml`, `controller.yaml`, `viewer.yaml`) and are not repeated here unless they explain a rule.

**Status:** the physics, body and joint control exist and run in the viewer. The reward, episode
and Gymnasium environment do not exist yet, so those sections say so.

## Timing

**Why it matters.** The controller gains are sized for the step sizes below. Changing a rate changes
whether a joint controller is stable.

- **Frame rate:** `world.sim_hz` (60). The viewer runs one control frame per tick, and the gait
  clock advances by `1 / sim_hz` per frame.
- **Physics substeps:** `world.physics_substeps` (8) per frame, so one physics step is
  `1 / (sim_hz · substeps)` s (about 2.1 ms).
- **Torques are recomputed every substep.** Targets are computed once per frame and held for all
  substeps. The integral term of a PID controller uses the substep length as its `dt`.
- **Episode length:** not defined yet. There is no episode or termination logic.

## Body

Built in `Simulation.setup` ([`sim/physics.py`](../src/humanoid_walk/sim/physics.py)) from
`humanoid.yaml`. The world is a 2D pymunk space in pixels (`pixels_per_meter` = 100). Torques given in
N·m are scaled by `ppm²` when applied, so all config values are in SI units.

**Links.** Ten rigid bodies, all with friction from the first joint entry (hip) in `humanoid.yaml`:

| Body        | Shape                                  | Mass (kg) | Size (m)                                   |
| ----------- | -------------------------------------- | --------- | ------------------------------------------ |
| Head        | circle                                 | 6.0       | radius `head_radius_m`                     |
| Torso       | vertical segment                       | 37.5      | `torso_length_m` long, `torso_width_m` wide |
| Arm (×2)    | vertical segment, hangs from the torso top | 3.75  | `arm_length_m` long, `arm_width_m` wide    |
| Thigh (×2)  | vertical segment                       | 7.5       | `leg_length_m / 2` long, `leg_width_m` wide |
| Shin (×2)   | vertical segment                       | 3.45      | `leg_length_m / 2` long, `leg_width_m` wide |
| Foot (×2)   | horizontal segment                     | 1.05      | `foot_length_m` long, `foot_width_m` wide  |

The masses must sum to `mass_kg` or loading the config fails. The ankle pivot sits `ankle_offset_m` behind the
foot centre. Bodies share a collision group, so the body parts do not collide with each other, only with
the ground.

**Joints.** Nine pin joints, each with a rotary limit joint. A joint angle is the child angle minus the
parent angle, and positive turns the child clockwise on screen (0 is the standing pose).

| Joint        | Child → parent  | + means                  | Limits (rad)   | Max torque (N·m) |
| ------------ | --------------- | ------------------------ | -------------- | ---------------- |
| `hip_l/r`    | thigh → torso   | leg swings backward      | −1.57 to 1.57  | 300              |
| `knee_l/r`   | shin → thigh    | knee flexes, shin back   | 0 to 2.0       | 200              |
| `ankle_l/r`  | foot → shin     | toes down                | −0.6 to 0.5    | 200              |
| `neck`       | head → torso    | head tilts forward       | −0.8 to 0.8    | 30               |
| `shoulder_l/r` | arm → torso   | arm swings backward      | −2.35 to 2.35  | 70               |

Setting `pin_torso: true` holds the torso upright with a gear joint. It is a debug option to test
propulsion without balance.

**Ground.** One static segment across the screen width with `world.floor_friction` and `world.ground_thickness_m`. Its top surface is half the thickness above `floor_y_px`. The humanoid spawns just above it, at `world.spawn_x_frac` of the screen width.

**Foot contact.** `Simulation.feet_on_ground()` says whether each foot is on the ground. A foot counts as down for `world.contact_hold_s` after its last pymunk contact, which bridges a single physics step where a resting contact drops out.

**Speed cap.** `world.max_body_speed_mps` limits the speed of every body, which keeps the solver stable if something goes wrong.

## Actions

An action is a **target angle in rad for each of the nine joints**, as a `JointTargets` dict keyed by
`JointName` (the order is the enum order: neck, shoulder_l, shoulder_r, hip_l, hip_r, knee_l, knee_r,
ankle_l, ankle_r).

`BodyController.compute_torques` turns targets into torques, one controller per joint, using the gains in
`controller.yaml`:

1. The target is clamped to the joint limits.
2. A PD or PID controller gives `Kp · error − Kd · velocity` (plus `Ki · ∫error` for PID).
3. The torque is clamped to the joint's `max_torque`.

Learned policies are expected to use the same targets. The scaling of a policy output to a target is not
defined yet.

## Observation

`Simulation.observe()` returns a flat list of 60 numbers: six per body, in this body order: head, torso,
left arm, right arm, left thigh, right thigh, left shin, right shin, left foot, right foot.

| Offset in a body's six | Meaning                                  | Unit |
| ---------------------- | ---------------------------------------- | ---- |
| 0, 1                   | x, y of the body centre (y points down)  | m    |
| 2                      | angle                                    | rad  |
| 3, 4                   | x, y velocity                            | m/s  |
| 5                      | angular velocity                         | rad/s |

This is a first draft and is unscaled. Positions are measured from the screen origin, not the humanoid, and
nothing reads the vector yet. Settle the final layout and scaling here before a controller indexes it.

The gait controller does not use this vector. It takes the torso angle and rate, the COM position and
velocity relative to each ankle, and the foot contact flags (see [gait.md](gait.md)).

## Rewards

Not implemented. Planned: forward velocity, an upright bonus, energy and smoothness penalties, and
termination on a fall.

## Terrain

Flat ground only.
