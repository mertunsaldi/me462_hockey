import importlib.util
import sys
from typing import Any

"""Utilities for loading client-supplied scenarios at runtime."""

# Import the base class from either package or top-level context.
try:  # when running ``python -m ball_example.app``
    from .scenarios import Scenario  # type: ignore
except Exception:  # fallback when ``app.py`` is executed directly
    from scenarios import Scenario  # type: ignore
    # Ensure ``ball_example.scenarios`` refers to the same module so that
    # client code importing ``ball_example.scenarios`` works.
    sys.modules.setdefault("ball_example.scenarios", sys.modules["scenarios"])


class ScenarioLoadError(Exception):
    pass


def load_scenario(path: str) -> Scenario:
    """Load a client scenario from a Python file.

    The module must define a ``ClientScenario`` class derived from
    :class:`ball_example.scenarios.Scenario`.
    """
    spec = importlib.util.spec_from_file_location("client_module", path)
    if spec is None or spec.loader is None:
        raise ScenarioLoadError(f"Cannot load module from {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    try:
        spec.loader.exec_module(module)  # type: ignore[arg-type]
    except Exception as e:
        raise ScenarioLoadError(f"Failed to import {path}: {e}")

    cls = getattr(module, "ClientScenario", None)
    if cls is None:
        raise ScenarioLoadError("ClientScenario class not found")
    if not issubclass(cls, Scenario):
        raise ScenarioLoadError("ClientScenario must subclass Scenario")
    try:
        return cls()
    except Exception as e:
        raise ScenarioLoadError(f"Could not instantiate ClientScenario: {e}")
