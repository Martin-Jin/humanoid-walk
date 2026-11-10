# Resources

A learning path for this project, from beginner to advanced. Work down the stages
in order. Each stage says what to be able to do before moving on. Sources are
official documentation, universities and papers where possible, otherwise popular
educational sites. Each link was checked to load when this page was written.

## Stage 1: Tools and neural network basics

Goal: draw something in pygame, simulate a falling body in pymunk, and know what a
neural network and a training loop are.

- [Pygame intro](https://pyga.me/docs/tutorials/en/intro-to-pygame.html) (pygame-ce docs) — a window, a loop and drawing. This is the viewer.
- [Pymunk overview](https://www.pymunk.org/en/latest/overview.html) and [examples](https://www.pymunk.org/en/latest/examples.html) (official) — bodies, shapes, joints and motors. The examples include a motorized, jointed body.
- [3Blue1Brown: neural networks](https://www.3blue1brown.com/topics/neural-networks) — a visual explanation of what a network is and how it learns.
- [PyTorch: Learn the Basics](https://pytorch.org/tutorials/beginner/basics/intro.html) (official) — tensors, a model and a training loop.

## Stage 2: Control basics

Goal: explain PID, and make a simple joint hold a target angle. Do this before any
RL. If you can't make the body stand by hand-tuning, learning will not fix it.

- [Understanding PID Control](https://www.youtube.com/playlist?list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y) (Brian Douglas, MATLAB) — the most popular short intro to PID.
- [Control Bootcamp](https://www.youtube.com/playlist?list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m) (Steve Brunton, University of Washington) — state space, stability and LQR, explained visually.
- [python-control](https://python-control.readthedocs.io/) (official) — PID and state-space tools in Python.

## Stage 3: RL basics

Goal: explain states, actions, rewards, policies and value functions, and run an
agent in an environment.

- [Hugging Face Deep RL course](https://huggingface.co/learn/deep-rl-course) — free and beginner friendly. Do the first units, up to a policy-gradient method.
- [Gymnasium: basic usage](https://gymnasium.farama.org/introduction/basic_usage/) (official) — the `reset` and `step` loop that `envs/` will follow.
- [Spinning Up, Part 1: Key Concepts in RL](https://spinningup.openai.com/en/latest/spinningup/rl_intro.html) (OpenAI) — the vocabulary used in every later paper and doc.
- [Karpathy: Pong from Pixels](https://karpathy.github.io/2016/05/31/rl/) — a short, readable walkthrough of a policy gradient from scratch.

## Stage 4: Policy gradients and PPO

Goal: understand every line of a PPO update, then run a trusted PPO on a walker.

- [Spinning Up, Part 3: Intro to Policy Optimization](https://spinningup.openai.com/en/latest/spinningup/rl_intro3.html) (OpenAI) — policy gradients, then advantages.
- [Spinning Up: PPO](https://spinningup.openai.com/en/latest/algorithms/ppo.html) (OpenAI) — the algorithm in one page.
- [Lilian Weng: Policy Gradient Algorithms](https://lilianweng.github.io/posts/2018-04-08-policy-gradient/) — a clear survey from REINFORCE to PPO and SAC.
- [Stable-Baselines3: getting started](https://stable-baselines3.readthedocs.io/en/master/guide/quickstart.html) (official) — train a trusted PPO in a few lines on your environment. If it can't learn the task, the environment or reward is the problem, not your PPO.
- [Gymnasium: creating a custom environment](https://gymnasium.farama.org/introduction/create_custom_env/) (official) — how to wrap your pymunk walker.
- [Gymnasium BipedalWalker](https://gymnasium.farama.org/environments/box2d/bipedal_walker/) and [Walker2d](https://gymnasium.farama.org/environments/mujoco/walker2d/) (official) — two 2D walkers with documented observations, actions and rewards. Copy their ideas for yours.

## Stage 5: Writing your own PPO

Goal: a hand-written PPO that matches the trusted baseline.

- [Proximal Policy Optimization (Schulman et al., 2017)](https://arxiv.org/abs/1707.06347) — the PPO paper.
- [Generalized Advantage Estimation (Schulman et al., 2015)](https://arxiv.org/abs/1506.02438) — the GAE paper. Needed for the `terminated` versus `truncated` bootstrapping.
- [The 37 implementation details of PPO](https://iclr-blog-track.github.io/2022/03/25/ppo-implementation-details/) (ICLR blog track) — the details the paper leaves out. Read it before writing `training/ppo.py`.
- [CleanRL](https://github.com/vwxyzjn/cleanrl) — single-file PPO to compare your code against, line by line.
- [Sutton & Barto, *Reinforcement Learning: An Introduction*](http://incompleteideas.net/book/the-book-2nd.html) — the textbook. Read the policy-gradient chapter when a detail confuses you.

## Stage 6: Walking-specific methods

Goal: reward design, and the classical walking models to compare against.

- [Emergence of Locomotion Behaviours in Rich Environments](https://arxiv.org/abs/1707.02286) (Heess et al., 2017) — walking from a simple reward plus varied terrain.
- [Learning to Walk in Minutes Using Massively Parallel Deep RL](https://arxiv.org/abs/2109.11978) (Rudin et al., 2021) — reward terms used for legged robots.
- [Underactuated Robotics](https://underactuated.csail.mit.edu/) (Tedrake, MIT) — free course. Use its chapters on the simple models of walking and on LQR.
- [`scipy.linalg.solve_continuous_are`](https://docs.scipy.org/doc/scipy/reference/generated/scipy.linalg.solve_continuous_are.html) (official) — the Riccati solver behind LQR.
- Search terms for topics without a stable link: *linear inverted pendulum model* and *Kajita ZMP preview control* (walking gaits), *Ijspeert central pattern generators review* (oscillator gaits).

## Stage 7: Optional, once walking works

- [Soft Actor-Critic](https://arxiv.org/abs/1801.01290) and [TD3](https://arxiv.org/abs/1802.09477) — off-policy methods. Try them if simulation is slow and PPO needs too many steps.
- [DeepMimic](https://arxiv.org/abs/1804.02717) (Peng et al., 2018) — learning from reference motion, if you want a more natural gait.

## Suggested order of work

1. Stages 1 and 2: stand the body up with PID, no RL.
2. Stage 3 and the Gymnasium wrapper from Stage 4: make the walker an environment.
3. Stage 4: train with Stable-Baselines3 to prove the reward works.
4. Stage 5: replace it with your own PPO.
5. Stage 6: add classical controllers and compare them on the same metrics.
