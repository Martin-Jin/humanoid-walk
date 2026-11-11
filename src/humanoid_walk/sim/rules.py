"""This file defines functions that determine how things in the simulation should behave."""

import pymunk


def limit_velocity(body, gravity, damping, dt):
    """Limit the velocity of a body to prevent it from going too fast."""
    max_velocity = 1000
    pymunk.Body.update_velocity(body, gravity, damping, dt)
    l = body.velocity.length
    if l > max_velocity:
        scale = max_velocity / l
        body.velocity = body.velocity * scale
