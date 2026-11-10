# Humanoid Walk

Humanoid Walk is a Python project on making a 2D humanoid walk. It compares
reinforcement learning (PPO) with classical control (PID, inverted-pendulum
methods, oscillator gaits) on one shared environment, and shows the result in a
pygame viewer.

The project is a skeleton. Nothing is implemented yet.

## Planned features

- 2D rigid-body humanoid simulated with `pymunk`, stepped at a fixed rate.
- Gymnasium environment with a shaped walking reward.
- Hand-written PPO training with checkpoints and resume.
- Classical controllers behind the same interface as the learned policy, so
  both are scored on identical metrics.
- Pygame viewer to watch a controller or a trained policy walk.

## Setup

See the [setup guide](docs/setup.md). It covers the CPU and NVIDIA CUDA installs.

## Layout

| Folder                  | Holds                                                         |
| ----------------------- | ------------------------------------------------------------- |
| `src/humanoid_walk/sim` | Physics world, body, terrain, observation vector. No pygame.  |
| `.../envs`              | Walking environment, rewards, vector env, Gymnasium adapter.  |
| `.../controllers`       | Classical controllers (PID, LQR/ZMP, CPG) and their interface.|
| `.../agents`            | Action codec, policy and actor-critic network.                |
| `.../training`          | PPO, rollout buffer, rolling stats, checkpoints.              |
| `.../evaluation`        | Gait metrics and the benchmark that compares controllers.     |
| `.../viewer`            | Pygame only: app, scenes, rendering and widgets.              |
| `.../utils`             | Seeding, device choice, logging and paths.                    |
| `configs/`, `scripts/`  | YAML settings and entry-point scripts.                        |
| `tests/`                | Unit tests, mirroring `src/`.                                 |
| `data/`, `checkpoints/`, `runs/` | Generated output, gitignored.                        |

## Documentation index

- [docs/setup.md](docs/setup.md) — install, CUDA and verify the checkout.
- [docs/hardware.md](docs/hardware.md) — what runs on the CPU and the GPU.
- [docs/simulation-spec.md](docs/simulation-spec.md) — body, actions, observation, rewards, timing.
- [docs/training-process.md](docs/training-process.md) — training loop and PPO.
- [docs/controllers.md](docs/controllers.md) — classical control and the shared interface.
- [docs/modules-reference.md](docs/modules-reference.md) — every module and folder.
- [docs/resources.md](docs/resources.md) — papers, courses and docs for learning each part.
