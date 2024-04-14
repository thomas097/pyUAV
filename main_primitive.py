import cv2
from pyuav.graphics.meshes import *
from pyuav.graphics.lighting import *
from pyuav.graphics.rendering import *
from pyuav.debugging import *
from pyuav.dynamics.quadcopter_primitive import QuadcopterPhysicsClient


QUADCOPTER_START_POS = np.array([15, 0, 15])
QUADCOPTER_DEST_POS = np.array([12, 5, 10])
BOEING_787_START_POS = np.array([0, 0, 0])
CAMERA_START_POS = np.array([17, 4, 13])

# Setup quadcopter
quadcopter = Mesh(
    file_path="assets\\low_poly_drone\\body.obj",
    material=Material("assets\\low_poly_drone\\diffuse.png"),
    position=QUADCOPTER_START_POS
    )

rotor_lf = Mesh(
    file_path="assets\\low_poly_drone\\rotor.obj",
    material=Material(diffuse="#dc8698"), # pink
    position=(0.126514, 0.028611, 0.250717),
    parent=quadcopter
    )

rotor_rf = Mesh(
    file_path="assets\\low_poly_drone\\rotor.obj",
    material=Material(diffuse="#dc8698"),
    position=(0.126514, 0.028611, -0.250717),
    parent=quadcopter
    )

rotor_lr = Mesh(
    file_path="assets\\low_poly_drone\\rotor.obj",
    material=Material(diffuse="#dc8698"),
    position=(-0.394567, 0.028611, 0.250717),
    parent=quadcopter
    )

rotor_rr = Mesh(
    file_path="assets\\low_poly_drone\\rotor.obj",
    material=Material(diffuse="#dc8698"),
    position=(-0.394567, 0.028611, -0.250717),
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


# Add physics body to model drone
physics_client = QuadcopterPhysicsClient(
    positions=[QUADCOPTER_START_POS]
)


# Render from drone's camera
renderer = Renderer(height=320, width=480)

for _ in range(99999):
    image = renderer.render(scene, camera, return_buffer=True)

    # Update state of quadcopter
    position, rotation = physics_client.control(index=0, target=target, heading=np.pi/2)

    quadcopter.set_position(position)
    quadcopter.set_rotation(rotation)

    # Make rotors turn for aesthetic purposes
    rotor_rf.rotate_y(-0.5)
    rotor_rr.rotate_y(0.5)
    rotor_lf.rotate_y(0.5)
    rotor_lr.rotate_y(-0.5)

    camera.set_lookat(position)

    cv2.imshow("", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    cv2.waitKey(1)
