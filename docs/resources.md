# Resources

Where to learn each part of the project from scratch. Each link was checked to
load when this page was written. Items are in a suggested reading order within
each section.

## Reinforcement learning fundamentals

- [Sutton & Barto, *Reinforcement Learning: An Introduction*](http://incompleteideas.net/book/the-book-2nd.html) — the standard textbook, free online. Read the chapters on policy gradients first.
- [OpenAI Spinning Up](https://spinningup.openai.com/en/latest/) — short, practical intro to deep RL, with PPO, TD3 and SAC explained.
- [Hugging Face Deep RL course](https://huggingface.co/learn/deep-rl-course) — hands-on, beginner friendly.
- [Berkeley CS285: Deep Reinforcement Learning](https://rail.eecs.berkeley.edu/deeprlcourse/) — lectures and homeworks, more advanced.

## PPO and the algorithms around it

- [Proximal Policy Optimization (Schulman et al., 2017)](https://arxiv.org/abs/1707.06347) — the PPO paper.
- [Generalized Advantage Estimation (Schulman et al., 2015)](https://arxiv.org/abs/1506.02438) — the GAE paper. Needed for the `terminated` versus `truncated` bootstrapping.
- [The 37 implementation details of PPO](https://iclr-blog-track.github.io/2022/03/25/ppo-implementation-details/) — the details the paper leaves out. Read before writing `training/ppo.py`.
- [CleanRL](https://github.com/vwxyzjn/cleanrl) — single-file PPO, SAC and TD3 to compare your code against.
- [Stable-Baselines3](https://stable-baselines3.readthedocs.io/) — a trusted baseline. Use it to check that your environment is learnable before blaming your own PPO.
- [Soft Actor-Critic](https://arxiv.org/abs/1801.01290) and [TD3](https://arxiv.org/abs/1802.09477) — off-policy alternatives if simulation turns out slow.

## Walking with RL: papers

- [Emergence of Locomotion Behaviours in Rich Environments (Heess et al., 2017)](https://arxiv.org/abs/1707.02286) — walking from a simple reward plus a varied terrain curriculum.
- [DeepMimic (Peng et al., 2018)](https://arxiv.org/abs/1804.02717) — learning from reference motion. Relevant if you add imitation later.
- [Learning to Walk in Minutes Using Massively Parallel Deep RL (Rudin et al., 2021)](https://arxiv.org/abs/2109.11978) — reward terms and training tricks for legged robots.

## Reference walking environments

Read how others define the observation, action and reward before writing yours.

- [Gymnasium Walker2d](https://gymnasium.farama.org/environments/mujoco/walker2d/) — a 2D walker with a documented reward.
- [Gymnasium BipedalWalker](https://gymnasium.farama.org/environments/box2d/bipedal_walker/) — a 2D walker in a Box2D physics engine, closest in spirit to this project.

## Classical control

- [Underactuated Robotics (Tedrake)](https://underactuated.csail.mit.edu/) — free online course. Covers walking, the inverted pendulum, LQR and optimization-based control.
- [Modern Robotics (Lynch & Park)](http://hades.mech.northwestern.edu/index.php/Modern_Robotics) — free book, videos and software. Covers kinematics, dynamics and control.
- [python-control](https://python-control.readthedocs.io/) — PID, state-space and LQR design in Python.
- [`scipy.linalg.solve_continuous_are`](https://docs.scipy.org/doc/scipy/reference/generated/scipy.linalg.solve_continuous_are.html) — the Riccati solver behind LQR.
- Search terms for topics without a stable link:
  - *Kajita ZMP preview control* and *linear inverted pendulum model* for walking gaits.
  - *Raibert hopper* and *SLIP model* for simple hopping and running.
  - *Ijspeert central pattern generators review* for oscillator gaits.
  - *Hybrid zero dynamics, Westervelt* for planar biped control theory.

## Simulation, environment and viewer

- [pymunk documentation](https://www.pymunk.org/en/latest/) and its [examples](https://www.pymunk.org/en/latest/examples.html) — rigid bodies, joints, motors and constraints. The examples include ragdolls and motorized joints.
- [pymunk source](https://github.com/viblo/pymunk) — read the code when the docs leave a gap.
- [Gymnasium: creating a custom environment](https://gymnasium.farama.org/introduction/create_custom_env/) — the interface `envs/` will expose.
- [pygame-ce documentation](https://pyga.me/docs/) — the viewer.
- [PyTorch tutorials](https://pytorch.org/tutorials/) — the network and training loop.

## Suggested order

1. Read Spinning Up's intro and the PPO paper, then the 37 implementation details.
2. Read how Gymnasium's BipedalWalker defines its observation and reward.
3. Build the pymunk body from the pymunk examples, and check it stands under a PID controller (control sanity check before any RL).
4. Wrap it as a Gymnasium environment and train a baseline with Stable-Baselines3 or CleanRL to prove the reward works.
5. Move to your own PPO, then compare it against the classical controllers.
