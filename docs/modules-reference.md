# Modules reference

Every module and folder. Update this file when folders or modules change.

## `src/humanoid_walk/`

- `__init__.py`: package version.
- `config.py`: loads the YAML files from `configs/` into frozen dataclasses (`Config`), and `validate_gait` rejects a gait the leg cannot reach.

## Folders

### `sim/`
Pure physics simulation. Must not import `pygame`.
- `physics.py`: `Simulation` builds the pymunk space, the ground and the humanoid, and exposes joint states, torque application, the COM, foot contacts and the observation vector. See [simulation-spec.md](simulation-spec.md).

### `controllers/`
Classical controllers and the shared controller interface. See [controllers.md](controllers.md).
- `joint_state.py`: `JointName` (the only joint names), `JointState`, `JointTargets`, `BodyState` and the left and right colours.
- `controller.py`: `P`, `PD`, `PI` and `PID` controllers and `make_controller`.
- `joint_control.py`: `JointController` (clamps target and torque), `BodyController` (all nine joints) and the builders that read `controller.yaml`.
- `gait.py`: `GaitController`, the foot-trajectory gait with start-up, balance, push-off and arm swing. See [gait.md](gait.md).

### `agents/`
Action codec, policy and actor-critic network. Only the `Policy` interface and `HoldPolicy` exist (`policy.py`).

### `viewer/`
Pygame only.
- `render/app.py`: `App`, the main loop (gait inputs, control step, drawing, reset).
- `render/game.py`: `main()`, the entry point that builds `App(load_config())`.
- `render/render.py`: `Renderer`, mouse dragging of bodies.
- `render/debug.py`: `print_walk_state`, periodic gait and joint printout.
- `ui/`: `JointPanel` (manual joint target sliders) and `Button`.
- `scenes/`: empty.

### `utils/`
- `seeding.py`, `device.py`, `log.py`, `paths.py`: seeding, device choice, logging and repository paths.

### Not started
- `envs/`: walking environment, rewards, vector env, Gymnasium adapter.
- `training/`: PPO, rollout buffer, rolling stats, checkpoints.
- `evaluation/`: gait metrics and the controller benchmark.

## Other folders

- `configs/`: `sim.yaml`, `humanoid.yaml`, `controller.yaml`, `gait.yaml`, `viewer.yaml`.
- `tests/`: unit tests, mirroring `src/`.
- `docs/`: documentation. `docs/resources/` holds the SVG figures used by the docs.
- `scripts/`: entry-point scripts (empty).
