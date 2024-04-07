import pygfx as gfx
from datatypes import *


class DirectionalLight(GfxObject):
    def __init__(
            self, 
            color: str = "#d6eec4",
            intensity: float = 3.0,
            cast_shadow: bool = True,
            position: Vector3f = (0, 0, 0),
            lookat: Vector3f = (0, -1, 0)
            ) -> None:
        
        target = gfx.WorldObject(visible=False)
        target.world.position = lookat
        
        self._instance = gfx.DirectionalLight(
            color=color, 
            intensity=intensity, 
            cast_shadow=cast_shadow, 
            target=target
            )
        
        # Initial pose
        self._instance.local.position = position


class AmbientLight(GfxObject):
    def __init__(
            self, 
            color: str = "#c0d6e4",
            intensity: float = 0.5
            ) -> None:
        
        self._instance = gfx.AmbientLight(color=color, intensity=intensity)
