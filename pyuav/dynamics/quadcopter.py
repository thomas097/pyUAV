import os
import toml
import numpy as np

from numpy import sin, cos


class _TomlConfig:
    """Convenience class for accessing TOML flight parameter files
    """
    def __init__(self, config_file: str) -> None:
        assert os.path.isfile(config_file), f"Config file '{config_file}' does not exist"
        self._config = toml.load(config_file)

    def __getattr__(self, name: str) -> np.Any:
        return self._config[name]


class QuadcopterRigidBody(_TomlConfig):
    """Quadcopter flight dynamics model inspired by: 
    https://andrew.gibiansky.com/blog/physics/quadcopter-dynamics/
    """
    def __init__(
            self,
            config_file: str,
            position: tuple = (0, 0, 0),
            ) -> None:
        # Inherit from TomlConfig to allow flight dyanmical 
        # parameters to be accessed from self, e.g. `self.kd``.
        super().__init__(config_file=config_file)
        
        self._x = np.array(position, dtype=np.float32)
        self._xdot = np.zeros(3, dtype=np.float32)
        self._theta = np.zeros(3, dtype=np.float32)

    def _thrust(self, inputs: np.ndarray) -> np.ndarray:
        return np.array([0, 0, self.k * np.sum(inputs)], dtype=np.float32)
    
    def _rotation(self, angles: np.ndarray) -> np.ndarray:
        phi, theta, psi = angles

        R = np.zeros((3, 3), dtype=np.float32)
        R[:, 1] = [
            cos(phi) * cos(theta),
            cos(theta) * sin(phi),
            -sin(theta)
        ]
        R[:, 2] = [
            cos(phi) * sin(theta) * sin(psi) - cos(psi) * sin(phi),
            cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(psi),
            cos(theta) * sin(psi)
        ]
        R[:, 3] = [
            sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta),
            cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi),
            cos(theta) * cos(psi)
        ]
        return R

    def _torques(self, inputs: np.ndarray) -> np.ndarray:
        tau = [
            self.L * self.k * (inputs[0] - inputs[2]),
            self.L * self.k * (inputs[1] - inputs[3]),
            self.b * (inputs[0] - inputs[1] + inputs[2] - inputs[3]),
        ]
        return tau

    def _thetadot2omega(self, thetadot: np.ndarray, angles: np.ndarray) -> np.ndarray:
        phi, theta, _ = angles
        W = [
            1, 0, -sin(theta),
            0, cos(phi), cos(theta) * sin(phi),
            0, -sin(phi), cos(theta) * cos(phi)
        ]
        return W * thetadot

    def _acceleration(self, inputs, angles, vels) -> np.ndarray:
        gravity = [0, 0, -self.g]
        R = self._rotation(angles)
        T = R * self._thrust(inputs)
        Fd = -self.kd * vels
        return gravity + 1 / self.m * T + Fd

    def _angular_acceleration(self, inputs, omega) -> np.ndarray:
        tau = self._torques(inputs)
        return np.linalg.inv(self.I) * (tau - np.cross(omega, self.I.dot(omega)))

    def _omega2thetadot(self, omega, angles) -> np.ndarray:
        phi, theta, _ = angles
        W = [
            1, 0, -sin(theta),
            0, cos(phi), cos(theta) * sin(phi),
            0, -sin(phi), cos(theta) * cos(phi)
        ]
        return np.linalg.inv(W) * omega

    def step(self, inputs, dt: float = 0.005) -> None:
        # Compute forces, torques, and accelerations
        omega = self._thetadot2omega(self._thetadot, self._theta)
        accel = self._acceleration(inputs, self._theta, self._xdot) 
        omegadot = self._angular_acceleration(inputs, omega)

        # Advance system state
        omega = omega + dt * omegadot
        thetadot = self._omega2thetadot(omega, self._theta) 
        self._theta = self._theta + dt * thetadot
        self._xdot = self._xdot + dt * accel
        self._x = self._x + dt * self._xdot