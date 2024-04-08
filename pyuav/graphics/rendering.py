import numpy as np
import pygfx as gfx
from pyuav.graphics.datatypes import *
from wgpu.gui.offscreen import WgpuCanvas


class Scene(GfxObject):
    def __init__(self, *objs: tuple[GfxObject]) -> None:
        self._instance = gfx.Scene()
        for obj in objs:
            self.add(obj)

    def add(self, obj: GfxObject) -> None:
        self._instance.add(obj.get_instance())


class Axes(GfxObject):
    def __init__(self, size: int = 5, thickness: int = 2) -> None:
        self._instance = gfx.AxesHelper(size=size, thickness=thickness)


class Grid(GfxObject):
    def __init__(self, size: int = 20, thickness: int = 1) -> None:
        self._instance = gfx.GridHelper(size=size, divisions=size, thickness=thickness)


class PerspectiveCamera(GfxObject):
    def __init__(
            self, 
            position: Vector3f = (0, 0, 0),
            rotation: Quaternion = (0, 0, 0, 1),
            fov: float = 55.0,
            parent: GfxObject = None
            ) -> None:
        
        self._instance = gfx.PerspectiveCamera(fov=fov)

        # [Optional] Parent to other object
        if parent is not None:
            self.parent_to(parent)

        # Set initial pose
        self.set_position(position, mode='local')
        self.set_rotation(rotation, mode='local')

    def parent_to(self, mesh: GfxObject) -> None:
        mesh.get_instance().add(self._instance, keep_world_matrix=False)

    def set_lookat(self, target: Vector3f, reference_up: Vector3f = (0, 1, 0)) -> None:
        self._instance.local.reference_up = reference_up
        self._instance.look_at(target)

    def set_position(self, position: Vector3f, mode: str = 'local') -> None:
        if mode == 'world':
            self._instance.world.position = position
        else:
            self._instance.local.position = position

    def set_rotation(self, rotation: Quaternion, mode: str = 'local') -> None:
        if mode == 'world':
            self._instance.world.rotation = rotation
        else:
            self._instance.local.rotation = rotation

    def get_position(self, mode: str = 'local') -> Vector3f:
        if mode == 'world':
            return self._instance.world.position
        else:
            return self._instance.local.position

    def get_rotation(self, mode: str = 'local') -> Vector3f:
        if mode == 'world':
            return self._instance.world.rotation
        else:
            return self._instance.local.rotation
    

class Renderer:
    """Renderer responsible for rendering game frames
    """
    def __init__(self, width: int = 640, height: int = 480) -> None:
        self._canvas = WgpuCanvas(size=(width, height), pixel_ratio=1)
        self._renderer = gfx.renderers.WgpuRenderer(self._canvas)
        print("Renderer.__init__() :: Initialized")

    def render(self, scene: Scene, camera: PerspectiveCamera, return_buffer: bool = False) -> np.ndarray:
        """Renders scene using specified camera.

        Args:
            scene (Scene):                  Scene to be rendered.
            camera (PerspectiveCamera):     Camera used for rendering.
            return_buffer (bool, optional): Whether to return the raw frame buffer with minimal antialiasing.

        Returns:
            np.ndarray: Rendered frame with shape (height, width, 4).
        """
        self._canvas.request_draw(lambda: self._renderer.render(scene.get_instance(), camera.get_instance()))
        frame = self._canvas.draw()

        if return_buffer:
            return self._renderer.snapshot()
        else:
            return np.asarray(frame)


    