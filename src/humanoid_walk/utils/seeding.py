"""Seeding helpers, so simulation and training runs are reproducible."""

import random

import numpy as np


def make_rng(seed: int | None) -> np.random.Generator:
    """Return a NumPy generator. Simulation code takes one of these, never global state."""
    return np.random.default_rng(seed)


def spawn_seeds(seed: int, count: int) -> list[int]:
    """Derive ``count`` independent integer seeds, for example one per worker environment."""
    children = np.random.SeedSequence(seed).spawn(count)
    return [int(child.generate_state(1)[0]) for child in children]


def seed_everything(seed: int) -> None:
    """Seed Python, NumPy and (if installed) PyTorch. Use at the start of a script."""
    random.seed(seed)
    np.random.seed(seed)
    try:
        import torch
    except ImportError:
        return
    torch.manual_seed(seed)
