import cv2
import pylinalg as la
from pyuav.graphics.meshes import *
from pyuav.graphics.lighting import *
from pyuav.graphics.rendering import *
from pyuav.debugging import *
from pyuav.dynamics.quadcopter_pyflyt import QuadcopterPhysicsClient


QUADCOPTER_START_POS = np.array([15, 0, 15])
QUADCOPTER_DEST_POS = np.array([15, 5, 10])
BOEING_787_START_POS = np.array([0, 0, 0])
CAMERA_START_POS = np.array([25, 15, 25])

# Setup quadcopter
quadcopter = Mesh(
    file_path="assets\\low_poly_drone\\body.obj",
    material=Material("assets\\low_poly_drone\\diffuse.png"),
    position=QUADCOPTER_START_POS
    )

rotor_lf = Mesh(
    file_path="assets\\low_poly_drone\\rotor.obj",
    material=Material(diffuse="#dc8698"), # pink
    position=(0.250717, 0.028611, 0.126514),
    parent=quadcopter
    )

rotor_rf = Mesh(
    file_path="assets\\low_poly_drone\\rotor.obj",
    material=Material(diffuse="#dc8698"),
    position=(-0.250717, 0.028611, 0.126514),
    parent=quadcopter
    )

rotor_lr = Mesh(
    file_path="assets\\low_poly_drone\\rotor.obj",
    material=Material(diffuse="#dc8698"),
    position=(0.250717, 0.028611, -0.394567),
    parent=quadcopter
    )

rotor_rr = Mesh(
    file_path="assets\\low_poly_drone\\rotor.obj",
    material=Material(diffuse="#dc8698"),
    position=(-0.250717, 0.028611, -0.394567),
    parent=quadcopter
    )


# Setup Boeing 787-8
boeing = Mesh(
    file_path="assets\\boeing-787\\boeing-787.obj",
    material=Material("assets\\boeing-787\\textures\\diffuse.png"),
    position=BOEING_787_START_POS
    )
    

# Create camera to view scene (can be attached to quadcopter via `parent` argument)
camera = PerspectiveCamera(position=CAMERA_START_POS)


# Setup lighting
ambient_light = AmbientLight()
direct_light = DirectionalLight()


# Add target to fly towards 
target = np.array(QUADCOPTER_DEST_POS)

target_cube = Mesh(
    file_path="assets\\cube\\cube.obj",
    material=Material(diffuse="#ff0000"),
    position=target
    )


# Add all meshes and lights to a Scene
scene = Scene(Grid(), ambient_light, direct_light, quadcopter, boeing, target_cube)


def XZY_to_XYZ(a: np.ndarray) -> np.ndarray:
    return np.array([a[0], a[2], a[1]], dtype=a.dtype)

def XYZ_to_XZY(a: np.ndarray) -> np.ndarray:
    return np.array([a[0], a[2], a[1]], dtype=a.dtype)

def XYZ_to_XZY_quat(q: np.ndarray) -> np.ndarray:
    q1 = np.array([-q[1], q[2], -q[0], q[3]], dtype=q.dtype) 
    q2 = np.array([0, np.sin(-np.pi/4), 0, np.cos(-np.pi/4)], dtype=q.dtype) 
    return la.quat_mul(q2, q1)


# Add physics body to model drone
physics_client = QuadcopterPhysicsClient(
    positions=[XZY_to_XYZ(QUADCOPTER_START_POS)]
)


# Render from drone's camera
renderer = Renderer(height=320, width=480)

for _ in range(99999):
    image = renderer.render(scene, camera, return_buffer=True)

    # Update state of quadcopter
    positions, rotations = physics_client.control(targets=XZY_to_XYZ(target)[np.newaxis], headings=[np.pi])

    quadcopter.set_position(XYZ_to_XZY(positions[0]))
    quadcopter.set_rotation(XYZ_to_XZY_quat(rotations[0]))

    camera.set_lookat(XYZ_to_XZY(positions[0]))

    cv2.imshow("", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    cv2.waitKey(1)
    print('ok')
