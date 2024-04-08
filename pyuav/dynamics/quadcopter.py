import os
import toml
import numpy as np
import pylinalg as la

from typing import Iterable, Any
from numpy import sin, cos


class RigidBody:
    """Convenience class for accessing TOML flight parameter files
    """
    def __init__(self, config_file: str) -> None:
        assert os.path.isfile(config_file), f"Config file '{config_file}' does not exist"
        self._config = toml.load(config_file)

    def __getattr__(self, name: str) -> Any:
        return self._config[name]


class QuadcopterRigidBody(RigidBody):
    """Simplified quadcopter flight dynamics model, heavily inspired 
    by Andrew Gibiansky's quadcopter dynamics model: 
    https://andrew.gibiansky.com/blog/physics/quadcopter-dynamics/
    """
    def __init__(
            self,
            config_file: str,
            position: Iterable = [0, 0, 0]
            ) -> None:
        """Initializes initial system state and sets up flight dynamics parameters.

        Args:
            config_file (str): Path to TOML file specifying quadcopter dynamics
            position (tuple):  Initial position of quadcopter in world coordinates
        """
        super().__init__(config_file=config_file)
        self._x = np.array(position, dtype=np.float32)
        self._xdot = np.zeros(3, dtype=np.float32)
        self._theta = np.zeros(3, dtype=np.float32)
        self._thetadot = np.zeros(3, dtype=np.float32) + np.deg2rad(2 * 300 * np.random.random(3) - 300)

    def _total_thrust(self, inputs: np.ndarray) -> np.ndarray:
        """Calculates total thrust in local coordinate frame given inputs.

        Args:
            inputs (np.ndarray): Control inputs to individual motors (4,)

        Returns:
            np.ndarray: Total thrust excerted in local Z-direction
        """
        return np.array([0, 0, self.k * np.sum(inputs)], dtype=np.float32)
    
    def _euler_to_matrix(self, angles: np.ndarray) -> np.ndarray:
        """Convenience function to convert ZYZ euler angles to rotation matrix

        Args:
            angles (np.ndarray): Current pitch, yaw and roll angles in radians

        Returns:
            np.ndarray: 3x3 rotation matrix
        """
        phi, theta, psi = angles

        R = np.zeros((3, 3), dtype=np.float32)
        R[:, 0] = [
            cos(phi) * cos(theta),
            cos(theta) * sin(phi),
            -sin(theta)
        ]
        R[:, 1] = [
            cos(phi) * sin(theta) * sin(psi) - cos(psi) * sin(phi),
            cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(psi),
            cos(theta) * sin(psi)
        ]
        R[:, 2] = [
            sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta),
            cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi),
            cos(theta) * cos(psi)
        ]
        return R

    def _torques(self, inputs: np.ndarray) -> np.ndarray:
        tau = np.array([
            self.L * self.k * (inputs[0] - inputs[2]),
            self.L * self.k * (inputs[1] - inputs[3]),
            self.b * (inputs[0] - inputs[1] + inputs[2] - inputs[3]),
        ], dtype=np.float32)
        return tau

    def _thetadot2omega(self, thetadot: np.ndarray, angles: np.ndarray) -> np.ndarray:
        phi, theta, _ = angles
        W = np.array([
            [1, 0, -sin(theta)],
            [0, cos(phi), cos(theta) * sin(phi)],
            [0, -sin(phi), cos(theta) * cos(phi)]
        ], dtype=np.float32)
        return W.dot(thetadot)

    def _acceleration(self, inputs: np.ndarray, angles: np.ndarray, xdot: np.ndarray) -> np.ndarray:
        """Recalculates acceleration vector from control inputs, orientation and velocity vector.

        Args:
            inputs (np.ndarray): Control inputs to motors
            angles (np.ndarray): Orientation of quadcopter expressed in ZYZ Euler angles
            xdot (np.ndarray):   Velocity vector in inertial frame

        Returns:
            np.ndarray: Acceleration in inertial frame
        """
        gravity = [0, 0, -self.g]
        R = self._euler_to_matrix(angles)
        T = R.dot(self._total_thrust(inputs))
        Fd = -self.kd * xdot
        return gravity + 1 / self.m * T + Fd

    def _angular_acceleration(self, inputs: np.ndarray, omega: np.ndarray) -> np.ndarray:
        """Computes angular acceleration from angular velocity and control inputs

        Args:
            inputs (np.ndarray): Control inputs to the individual motors (4,)
            omega (np.ndarray): Angular velocity vector

        Returns:
            np.ndarray: Angular acceleration in inertial frame
        """
        tau = self._torques(inputs)
        I = np.diag(self.I)
        return np.linalg.inv(I).dot(tau - np.cross(omega, I.dot(omega)))

    def _omega2thetadot(self, omega, angles) -> np.ndarray:
        phi, theta, _ = angles
        W = np.array([
            [1, 0, -sin(theta)],
            [0, cos(phi), cos(theta) * sin(phi)],
            [0, -sin(phi), cos(theta) * cos(phi)]
        ], dtype=np.float32)
        return np.linalg.inv(W).dot(omega)

    def step(self, inputs, dt: float = 0.005) -> None:
        # Compute forces, torques, and accelerations
        omega = self._thetadot2omega(self._thetadot, self._theta)
        accel = self._acceleration(inputs, self._theta, self._xdot) 
        omegadot = self._angular_acceleration(inputs, omega)

        # Advance system state
        omega = omega + dt * omegadot
        self._thetadot = self._omega2thetadot(omega, self._theta) 
        self._theta = self._theta + dt * self._thetadot
        self._xdot = self._xdot + dt * accel
        self._x = self._x + dt * self._xdot

        return np.copy(self._x)


if __name__ == '__main__':
    model = QuadcopterRigidBody("pyuav\\dynamics\\simple_quadcopter.toml")
    for i in range(100):
        position = model.step([5.4e5] * 4)
        print(position)