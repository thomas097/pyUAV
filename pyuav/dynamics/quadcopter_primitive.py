import numpy as np
import pylinalg as la
from pyquaternion import Quaternion
from typing import Iterable

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
        self._num_drones = num_drones

        self._pos = np.array(positions, dtype=np.float32)
        self._vel = np.zeros((self._num_drones, 3), dtype=np.float32)
        self._rot = Quaternion(w=1, x=0, y=0, z=0)
        
        print('QuadcopterPhysics.__init__() :: Initialized')

    def _cliplength(self, matrix: np.ndarray, max_: float = 1.0, axis: int = -1) -> np.ndarray:
        norm = np.linalg.norm(matrix, axis=-1, keepdims=True)
        norm[norm < max_] = 1.0
        return matrix / norm
    
    def _normalize(self, matrix: np.ndarray) -> np.ndarray:
        norm = np.linalg.norm(matrix, axis=-1, keepdims=True)
        norm[norm < 1e-4] = 1.0
        return matrix / norm
    
    def _basis_to_quaternion(self, forw: np.ndarray, upv: np.ndarray) -> np.ndarray:
        upv = self._normalize(upv)
        side = self._normalize(np.cross(forw, upv, axis=-1))
        forw = np.cross(upv, side, axis=-1)
        matrix = np.array([forw, upv, side], dtype=np.float32).T
        x, y, z, w = la.vec_normalize(la.quat_from_mat(matrix))
        return Quaternion(w=w, x=x, y=y, z=z)
        
    def control(self, index: int, target: np.ndarray, heading: Iterable[float], dt: float = 0.05) -> None:
        assert 0 <= index < self._num_drones

        ####################
        #     Rotation
        ####################

        # Calculate desired orientation while approaching target
        direction = self._cliplength(target - self._pos[index], max_=1.0)

        upwards = direction[1] > 0.0

        if upwards:
            approach_rotation = self._basis_to_quaternion(
                forw=direction,
                upv=self._normalize(1.5 * direction + np.array([0, 1, 0], dtype=np.float32))
            )
        else:
            approach_rotation = self._basis_to_quaternion(
                forw=direction,
                upv=self._normalize(1.5 * -direction + np.array([0, 1, 0], dtype=np.float32))
            )

        # Calculate desired orientation when target is reached (correct heading)
        destination_rotation = self._basis_to_quaternion(
            forw=np.array([np.cos(heading + 1e-3), 0, np.sin(heading + 1e-3)]),
            upv=np.array([0, 1, 0], dtype=np.float32)
        )

        # Interpolate between rotations based on proximity to target
        prox = np.exp(-np.linalg.norm(target - self._pos[index]))
        desired_rotation = Quaternion.slerp(approach_rotation, destination_rotation, prox)
        self._rot = Quaternion.slerp(self._rot, desired_rotation, 0.5 * dt)

        rotation = np.array([self._rot.x, self._rot.y, self._rot.z, self._rot.w])

        ####################
        #     Position
        ####################

        # Calculate desired direction of flight
        self._vel[index] = self._cliplength(0.98 * self._vel[index] + 0.3 * dt * direction, max_=3.0)
        self._pos[index] += dt * self._vel[index]
        
        return np.copy(self._pos[index]), np.array(rotation)
        

if __name__ == '__main__':
    qc = QuadcopterPhysicsClient(
        num_drones=1,
        positions=[(0, 0, 0)],
        rotations=[(0, 0, 0, 1)]
    )

    # Target location to fly drone to
    target = np.array([1, 0, 0.5], dtype=np.float32)

    while True:
        position, rotation = qc.control(index=0, target=target, heading=0)
        print(position, rotation, np.linalg.norm(target - position))