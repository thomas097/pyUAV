import cv2
from pyuav.graphics.meshes import *
from pyuav.graphics.lighting import *
from pyuav.graphics.rendering import *
from pyuav.debugging import *
from pyuav.entities import Quadcopter


quadcopter = Quadcopter(position=(15, 0, 15))

# Define destination to fly towards
target_cube = Mesh(
    file_path="assets\\cube\\cube.obj",
    material=Material(diffuse="#ff0000"),
    position=(10, 10, 10)
    )

# Spawn aircraft
boeing = Mesh(
    file_path="assets\\boeing-787\\boeing-787.obj",
    material=Material("assets\\boeing-787\\textures\\diffuse.png"),
    position=(0, 0, 0) # centered on the ground
    )

# Camera to view scene (set to quadcopter.camera to use onboard camera!)
camera = PerspectiveCamera(position=(17, 9, 13))

# Setup lighting
ambient_light = AmbientLight()
direct_light = DirectionalLight()

# Add meshes, lights, etc. to scene
scene = Scene(Grid(), ambient_light, direct_light, quadcopter.body_and_rotors, boeing, target_cube)

# Render from drone's camera
renderer = Renderer(height=320, width=480)

for _ in range(99999):
    image = renderer.render(scene, camera, return_buffer=True)

    # Update state of quadcopter
    quadcopter.control(target=target_cube.get_position(), heading=np.pi)

    camera.set_lookat(quadcopter.get_position())

    cv2.imshow("", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    cv2.waitKey(1)
