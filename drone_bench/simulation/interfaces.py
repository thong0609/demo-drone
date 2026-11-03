"""Structural interfaces for plugging alternative controllers into the runner."""
from typing import Protocol


class OuterController(Protocol):
    def commands(self, positions: list[list[float]], active: list[int], *, time: float = 0.0, states=None) -> tuple[dict, dict]:
        """Return velocity commands and per-agent geometry with 'clearance'."""
        ...


class InnerController(Protocol):
    def initial_state(self, p, v, vc) -> list[float]:
        """State must start with [x,y,z,vx,vy,vz]; remaining entries are private."""
        ...

    def step(self, t, state, vc, agent, dt) -> list[float]: ...

    def rhs(self, t, state, vc, agent, diagnostics=False):
        """Diagnostics: input, tracking_error, disturbance_estimate, observer_residual_truth."""
        ...
