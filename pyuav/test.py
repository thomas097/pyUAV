from meshes import *
from lighting import *
from rendering import *

# Setup quadcopter
quadcopter = Mesh(
    file_path="assets\\low_poly_drone\\body.obj",
    material=Material("assets\\low_poly_drone\\diffuse.png"),
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

# Move drone body around
quadcopter.set_position(position=(-.2, 1.0, .3), mode='world')
quadcopter.set_rotation(rotation=(0.2, 0.3, 0.1, 0.9), mode='world')

# Attach camera to quadcopter
camera = PerspectiveCamera(position=(0.0, -0.1, -0.1), parent=quadcopter)

# Setup lighting
ambient_light = AmbientLight()
direct_light = DirectionalLight()

# Render
scene = Scene(Grid(), ambient_light, direct_light, quadcopter)

renderer = Renderer()
image = renderer.render(scene, camera, return_buffer=True)

import matplotlib.pyplot as plt
plt.imshow(image)
plt.show()