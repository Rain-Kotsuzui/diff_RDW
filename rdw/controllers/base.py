from __future__ import annotations

from abc import ABC, abstractmethod

from rdw.models import Observation, RedirectionCommand


class Redirector(ABC):
    """Required interface for every redirected-walking algorithm."""

    @property
    @abstractmethod
    def name(self) -> str:
        """Unique algorithm name used by experiments."""

    @abstractmethod
    def reset(self) -> None:
        """Clear controller state before a simulation."""

    @abstractmethod
    def compute(self, observation: Observation) -> RedirectionCommand:
        """Return one command without mutating the observation."""
