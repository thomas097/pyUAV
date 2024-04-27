from pyuav.graphics.meshes import Mesh, Material
from pyuav.graphics.rendering import PerspectiveCamera
from pyuav.dynamics.quadcopter_primitive import QuadcopterPhysicsClient as PrimitiveQuadcopterPhysicsClient
from pyuav.dynamics.quadcopter_pyflyt import QuadcopterPhysicsClient as PyFlytQuadcopterPhysicsClient
from pyuav.graphics.datatypes import Vector3f, Quaternion

# Files
MODEL_FILEPATH = "assets\\low_poly_drone\\body.obj"
TEXTURE_FILEPATH = "assets\\low_poly_drone\\diffuse.png"

ROTOR_FILEPATH = "assets\\low_poly_drone\\rotor.obj"
ROTOR_COLOR = "#333333" # dark gray


class Quadcopter:
    def __init__(
            self, 
            position: Vector3f = (0, 0, 0),
            rotation: Quaternion = (0, 0, 0, 1),
            mode: str = 'primitive'
            ) -> None:
        
        # UAV body
        self._body = Mesh(
            file_path=MODEL_FILEPATH,
            material=Material(TEXTURE_FILEPATH),
            position=position,
            rotation=rotation,
            )
        
        # Rotors
        create_rotor = lambda rel_position: Mesh(
            file_path=ROTOR_FILEPATH, 
            material=Material(diffuse=ROTOR_COLOR), 
            position=rel_position, 
            parent=self._body
            )
        
        self._rotor_lf = create_rotor((0.25, 0.0, 0.25))
        self._rotor_lr = create_rotor((0.25, 0.0, -0.25))
        self._rotor_rf = create_rotor((-0.25, 0.0, 0.25))
        self._rotor_rr = create_rotor((-0.25, 0.0, -0.25))
        
        # Whether to attach a camera to the UAV's body
        self._camera = PerspectiveCamera(
            position=position,
            parent=self._body
            )
        
        # Add rigid body physics to body
        if mode == 'primitive':
            self._physics_client = PrimitiveQuadcopterPhysicsClient(positions=[position], rotations=[rotation])
        elif mode == 'pyflyt':
            self._physics_client = PyFlytQuadcopterPhysicsClient(positions=[position], rotations=[rotation])
        else:
            raise Exception(f"mode '{mode}' not understood. Choose from 'pyflyt' or 'primitive' (default).")

    @property
    def body_and_rotors(self) -> Mesh:
        return self._body
    
    @property
    def camera(self) -> PerspectiveCamera:
        return self._camera
    
    def control(self, target: Vector3f, heading: float) -> None:
        new_position, new_rotation = self._physics_client.control(index=0, target=target, heading=heading)

        self._body.set_position(new_position)
        self._body.set_rotation(new_rotation)

        # Make rotors turn for aesthetic purposes
        self._rotor_rf.rotate_y(-1)
        self._rotor_rr.rotate_y(1)
        self._rotor_lf.rotate_y(1)
        self._rotor_lr.rotate_y(-1)

    def get_position(self) -> Vector3f:
        return self._body.get_position()
    
    def get_rotation(self) -> Vector3f:
        return self._body.get_rotation()