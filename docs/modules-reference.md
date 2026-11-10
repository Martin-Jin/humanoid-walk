# Modules reference

Every module and folder. Update this file when folders or modules change.

## `src/humanoid_walk/`

- `__init__.py` — package version.
- `config.py` — loads YAML files from `configs/`.

## Folders

- `sim/` — pure physics simulation. Must not import `pygame`.
- `envs/` — walking environment, rewards, vector env, Gymnasium adapter.
- `controllers/` — classical controllers and the shared controller interface.
- `agents/` — action codec, policy and actor-critic network.
- `training/` — PPO, rollout buffer, rolling stats, checkpoints.
- `evaluation/` — gait metrics and the controller benchmark.
- `viewer/` — pygame only. Subfolders: `render/`, `ui/`, `scenes/`.
- `utils/` — `seeding.py`, `device.py`, `log.py`, `paths.py`.
