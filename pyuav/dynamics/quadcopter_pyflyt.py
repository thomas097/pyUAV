import numpy as np
import pylinalg as la
from typing import Iterable
from PyFlyt.core import Aviary

class QuadcopterPhysicsClient:
    def __init__(
            self, 
            num_drones: int = 1,
            positions: Iterable = [[0, 0, 0]], 
            rotations: Iterable = [[0, 0, 0, 1]]
            ) -> None:
        """Implementation of the UAV dynamics using PyFlyt, modelled after the Crazyflie 2.0 nano-quadcopter.

        Args:
            num_drones (int): Total number of drones to simulate.
            positions (Iterable): Positions of quadcopters as a matrix of shape (num_drones, 3).
            rotations (Iterable): Rotations of quadcopters as a quaternion matrix of shape (num_drones, 4)
        """
        assert len(positions) == len(rotations) == num_drones, 'Number of positions and rotations must be equal to number of drones'

        # Implements drone physics model
        self._base = Aviary(
            start_pos=np.array(positions, dtype=np.float32),
            start_orn=np.array([la.quat_to_euler(r) for r in rotations], dtype=np.float32),
            drone_type='quadx',
            render=False
        ) 
        self._base.set_mode([7] * num_drones) # == (x, y, yaw, z)

        self._num_drones = num_drones
        self._prev_setpoints = None
        print('QuadcopterPhysics.__init__() :: Initialized')

        
    def control(self, targets: np.ndarray, headings: Iterable[float]) -> None:
        assert len(targets) == len(headings) == self._num_drones

        # Define setpoint as XYHZ
        x, y, z = targets.T
        h = np.array(headings, dtype=np.float32)
        setpoints = np.c_[x, y, h, z]

        # Update setpoint if target or heading changed
        if setpoints is not self._prev_setpoints:
            self._prev_setpoints = setpoints
            self._base.set_all_setpoints(setpoints=setpoints)

        # Advance simulation state
        self._base.step()

        # Get position and quaternion rotations
        positions = np.zeros((self._num_drones, 3), dtype=np.float32)
        rotations = np.zeros((self._num_drones, 4), dtype=np.float32)
        for i, state in enumerate(self._base.all_states):
            positions[i, :] = state[3]
            rotations[i, :] = la.quat_from_euler(state[1])

        return positions, rotations
        

if __name__ == '__main__':
    qc = QuadcopterPhysicsClient(
        num_drones=1,
        positions=[(0, 0, 0)],
        rotations=[(0, 0, 0, 1)]
    )

    # Target location to fly drone to
    target = np.array([[5, 0, 3]], dtype=np.float32)

    while True:
        position, rotation = qc.control(targets=target, headings=[0])
        print(position, rotation, np.linalg.norm(target - position))