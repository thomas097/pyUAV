import cv2
from pyuav.debugging import *
from pyuav.entities import Quadcopter
from pyuav.graphics.meshes import *
from pyuav.graphics.lighting import *
from pyuav.graphics.rendering import *


quadcopter1 = Quadcopter(position=(15, 0, 15))
quadcopter2 = Quadcopter(position=(25, 3, 10))

# Define destination to fly towards
target_cube = Mesh(
    file_path="assets\\cube\\cube.obj",
    material=Material(diffuse="#ff0000"),
    position=(10, 10, 10)
    )

boeing = Mesh(
    file_path="assets\\boeing-787\\boeing-787.obj",
    material=Material("assets\\boeing-787\\textures\\diffuse.png"),
    position=(0, 0, 0) # centered on the ground
    )

camera = PerspectiveCamera(position=(17, 9, 13))

ambient_light = AmbientLight()
direct_light = DirectionalLight()

scene = Scene(Grid(), ambient_light, direct_light, quadcopter1.body_and_rotors, 
              quadcopter2.body_and_rotors, boeing, target_cube)

# Render from drone's camera
renderer = Renderer(height=320, width=480)

for _ in range(99999):
    image = renderer.render(scene, camera, return_buffer=True)

    # Update state of quadcopter
    quadcopter1.control(target=target_cube.get_position(), heading=np.pi)
    quadcopter2.control(target=target_cube.get_position(), heading=np.pi)

    # Always look at quadcopter 1
    camera.set_lookat(quadcopter1.get_position())

    cv2.imshow("", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    cv2.waitKey(1)
