import cv2
from pyuav.graphics.meshes import *
from pyuav.graphics.lighting import *
from pyuav.graphics.rendering import *
from pyuav.debugging import *
from pyuav.dynamics.quadcopter_pyflyt import QuadcopterPhysicsClient

# Setup quadcopter
quadcopter = Mesh(
    file_path="assets\\low_poly_drone\\body.obj",
    material=Material("assets\\low_poly_drone\\diffuse.png")
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


## Setup Boeing 787-8
# boeing = Mesh(
#     file_path="assets\\boeing-787\\boeing-787.obj",
#     material=Material("assets\\boeing-787\\textures\\diffuse.png"),
#     )
    

# Create camera to view scene (can be attached to quadcopter via `parent` argument)
camera = PerspectiveCamera(position=(10, 20, 10))
camera.set_lookat((2, 0, 2))


# Setup lighting
ambient_light = AmbientLight()
direct_light = DirectionalLight()


# Add target cubes to fly towards and look at
target_cube = Mesh(
    file_path="assets\\cube\\cube.obj",
    material=Material(diffuse="#ff0000"),
    )

# Add physics body to model drone
physics_client = QuadcopterPhysicsClient()


# Add all meshes and lights to a Scene
scene = Scene(Grid(), Axes(35), ambient_light, direct_light, quadcopter, target_cube)


# Define target to fly to
target = np.array([0, 3, 10], dtype=np.float32)


def XZY_to_XYZ(a: np.ndarray) -> np.ndarray:
    return np.array([a[0], a[2], a[1]], dtype=a.dtype)

def XYZ_to_XZY(a: np.ndarray) -> np.ndarray:
    return np.array([a[0], a[2], a[1]], dtype=a.dtype)

def XYZ_to_XZY_quat(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], q[2], q[1], q[3]], dtype=q.dtype)


# Render from drone's camera
renderer = Renderer()

for _ in range(99999):
    image = renderer.render(scene, camera, return_buffer=True)

    # Update state of quadcopter
    positions, rotations = physics_client.control(targets=XZY_to_XYZ(target)[np.newaxis], headings=[np.pi])

    quadcopter.set_position(XYZ_to_XZY(positions[0]))
    quadcopter.set_rotation(XYZ_to_XZY_quat(rotations[0]))

    cv2.imshow("", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    cv2.waitKey(1)
