import os
import trimesh
import pygfx as gfx
import numpy as np
import pylinalg as la
import imageio.v3 as iio
from pyuav.graphics.datatypes import *


class Material(GfxObject):
    def __init__(
            self, 
            tex_file: str = None, 
            shininess: float = 30, 
            diffuse: str = "#000",
            emissive: str = "#000",
            specular: str = "#494949"
            ) -> None:

        if tex_file is not None:
            self._instance = gfx.MeshPhongMaterial(
                map=self._load_texture(tex_file),
                shininess=shininess,
                emissive=emissive,
                specular=specular
                )
        else:
            self._instance = gfx.MeshPhongMaterial(
                color=diffuse,
                shininess=shininess,
                emissive=emissive,
                specular=specular
            )

    def _load_texture(self, file_path: str) -> gfx.Texture:
        assert os.path.isfile(file_path), f"Texture file '{file_path}' does not exist"
        image = iio.imread(file_path).astype("float32") / 255
        return gfx.Texture(image, dim=2)


class Mesh(GfxObject):
    def __init__(
            self, 
            file_path: str, 
            material: Material,
            position: Vector3f = (0, 0, 0),
            rotation: Quaternion = (0, 0, 0, 1),
            parent: GfxObject = None
            ) -> None:
        assert os.path.isfile(file_path), f"3D model file '{file_path}' does not exist"

        # Create mesh object with material
        self._instance = gfx.Mesh(
            geometry=gfx.geometry_from_trimesh(
                trimesh.load(file_path)
            ),
            material=material.get_instance()
        )

        # [Optional] Parent to other object
        if parent is not None:
            self.parent_to(parent)

        # Set initial pose
        self.set_position(position, mode='local')
        self.set_rotation(rotation, mode='local')
        print(f'Mesh.__init__() :: loaded {file_path}')

    def parent_to(self, mesh: GfxObject) -> None:
        mesh.get_instance().add(self._instance, keep_world_matrix=False)

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

    def rotate_y(self, angle: float, mode: str = 'local') -> None:
        qrot = np.array([0, np.sin(angle / 2), 0, np.cos(angle / 2)])
        if mode == 'world':
            self._instance.world.rotation = la.quat_mul(qrot, self._instance.world.rotation)
        else:
            self._instance.local.rotation = la.quat_mul(qrot, self._instance.local.rotation)

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