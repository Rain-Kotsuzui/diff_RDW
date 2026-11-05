from __future__ import annotations

import importlib
import pkgutil
from collections.abc import Callable

from rdw.controllers.base import Redirector

ControllerFactory = Callable[[], Redirector]


def discover_controllers() -> dict[str, ControllerFactory]:
    """Discover algorithm modules in this package."""
    package = importlib.import_module("rdw.controllers")
    factories: dict[str, ControllerFactory] = {}

    for module_info in pkgutil.iter_modules(package.__path__):
        if module_info.name in {"base", "registry"} or module_info.name.startswith("_"):
            continue

        module = importlib.import_module(f"{package.__name__}.{module_info.name}")
        factory = module.CONTROLLER_FACTORY
        controller = factory()
        if not isinstance(controller, Redirector):
            raise TypeError(
                f"{module.__name__}.CONTROLLER_FACTORY must create a Redirector."
            )
        if controller.name in factories:
            raise ValueError(f"Duplicate controller name: {controller.name}")
        factories[controller.name] = factory

    return dict(sorted(factories.items()))
