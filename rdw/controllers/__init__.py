"""Replaceable redirected-walking algorithms.

Every algorithm module must define ``CONTROLLER_FACTORY``.
"""

from rdw.controllers.registry import discover_controllers

__all__ = ["discover_controllers"]
