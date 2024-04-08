import cv2
from pyuav.graphics.meshes import *
from pyuav.graphics.lighting import *
from pyuav.graphics.rendering import *
from pyuav.debugging import *

# Setup quadcopter
quadcopter = Mesh(
    file_path="assets\\low_poly_drone\\body.obj",
    material=Material("assets\\low_poly_drone\\diffuse.png"),
    position=(15, 3, 15),
    rotation=(0.2, 0.3, 0.1, 0.9)
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
    )
    

# Attach camera to quadcopter
# camera = PerspectiveCamera(position=(0.0, -0.1, -0.1), parent=quadcopter) # attched to drone body
camera = PerspectiveCamera(position=(30, 8, 35))
camera.set_lookat((5, 3, 5))

# Setup lighting
ambient_light = AmbientLight()
direct_light = DirectionalLight()

## Add target cubes to fly towards and look at
# target_cube_red = Mesh(
#     file_path="assets\\cube\\cube.obj",
#     material=Material(diffuse="#ff0000"),
#     )

# target_cube_blue = Mesh(
#     file_path="assets\\cube\\cube.obj",
#     material=Material(diffuse="#00ffff"),
#     )

# target_sampler1 = DebugTarget()
# target_sampler2 = DebugTarget()

# Add all meshes, lights, etc into a Scene
scene = Scene(Grid(), Axes(35), ambient_light, direct_light, quadcopter, boeing)

# Render from drone's camera
renderer = Renderer()

for _ in range(99999):
    image = renderer.render(scene, camera, return_buffer=True)

    # Sample target to fly towards and look at
    # target1 = target_sampler1.get_target()
    # target_cube_red.set_position(target1)

    # target2 = target_sampler2.get_target()
    # target_cube_blue.set_position(target2)

    # Update state of quadcopter
    # position, rotation = quadcopter_rigid_body.step(destination=target1, lookat=target2)
    # quadcopter.set_position(position)
    # quadcopter.set_rotation(rotation)

    cv2.imshow("", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    cv2.waitKey(1)
