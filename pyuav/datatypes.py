from typing import Iterable
import pygfx as gfx

Vector3f = Iterable[float]
Quaternion = Iterable[float]

class GfxObject:
    """Base class from which all wrapper classes of PyGFX internal objects must inherit
    """
    def __init__(self) -> None:
        self._instance = None

    def get_instance(self) -> gfx.WorldObject:
        return self._instance
        