# Humanoid Walk

Humanoid Walk is a Python project on making a 2D humanoid walk. It compares
reinforcement learning (PPO) with classical control (PID, inverted-pendulum
methods, oscillator gaits) on one shared environment, and shows the result in a
pygame viewer.

**Status.** The physics simulation, the joint controllers, a hand-written gait
controller and the pygame viewer exist. The humanoid walks forward at about
0.3 m/s and recovers from moderate pushes, until it reaches the end of the
ground. The Gymnasium environment, rewards, PPO training and evaluation are not
started.

## Features

- 2D rigid-body humanoid simulated with `pymunk`, stepped at a fixed rate. *(done)*
- PD and PID joint controllers, and a foot-trajectory gait controller with balance
  feedback. *(done)*
- Pygame viewer with a manual joint panel, a reset button and mouse dragging. *(done)*
- Gymnasium environment with a shaped walking reward. *(planned)*
- Hand-written PPO training with checkpoints and resume. *(planned)*
- Classical controllers behind the same interface as the learned policy, so
  both are scored on identical metrics. *(planned)*

## Setup and run

See the [setup guide](docs/setup.md). It covers the CPU and NVIDIA CUDA installs.

Start the viewer from the repository root with the virtual environment active:

```powershell
py -m humanoid_walk.viewer.render.game
```

## Layout

| Folder                  | Holds                                                         |
| ----------------------- | ------------------------------------------------------------- |
| `src/humanoid_walk/sim` | Physics world, body, ground, observation vector. No pygame.   |
| `.../envs`              | Walking environment, rewards, vector env, Gymnasium adapter. *(empty)* |
| `.../controllers`       | Joint controllers, the gait controller and joint names.       |
| `.../agents`            | Action codec, policy and actor-critic network. *(policy only)* |
| `.../training`          | PPO, rollout buffer, rolling stats, checkpoints. *(empty)*    |
| `.../evaluation`        | Gait metrics and the benchmark that compares controllers. *(empty)* |
| `.../viewer`            | Pygame only: app, rendering and widgets.                      |
| `.../utils`             | Seeding, device choice, logging and paths.                    |
| `configs/`, `scripts/`  | YAML settings and entry-point scripts.                        |
| `tests/`                | Unit tests, mirroring `src/`.                                 |
| `data/`, `checkpoints/`, `runs/` | Generated output, gitignored.                        |

## Documentation index

- [docs/setup.md](docs/setup.md): open it to install the project, set up CUDA, run the viewer and check the checkout.
- [docs/hardware.md](docs/hardware.md): open it to see what runs on the CPU and the GPU (mostly to be filled in once training exists).
- [docs/simulation-spec.md](docs/simulation-spec.md): open it for the body, joints, timing, actions and observation layout.
- [docs/training-process.md](docs/training-process.md): open it for the training loop and PPO (to be filled in once training exists).
- [docs/controllers.md](docs/controllers.md): open it to see how targets become torques and which controllers exist.
- [docs/gait.md](docs/gait.md): open it to understand or tune the gait controller (foot path, start-up, push-off, balance, arms).
- [docs/modules-reference.md](docs/modules-reference.md): open it to find what each module and folder holds.
- [docs/resources.md](docs/resources.md): open it for papers, courses and docs for learning each part.
