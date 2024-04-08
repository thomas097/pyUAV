import numpy as np

class DebugTarget:
    def __init__(
            self,
            x_range: tuple[float, float] = (5, 10),
            y_range: tuple[float, float] = (1, 3),
            z_range: tuple[float, float] = (5, 10),
            reset_freq: int = 200
            ) -> None:
        
        self._x_range = x_range
        self._y_range = y_range
        self._z_range = z_range
        self._reset_freq = reset_freq
        self._i = 0

        # Sample initial target
        self._target = self._sample_target()

    def _sample_target(self) -> np.ndarray:
        x = np.random.uniform(*self._x_range)
        y = np.random.uniform(*self._y_range)
        z = np.random.uniform(*self._z_range)
        return np.array([x, y, z], dtype=np.float32)
        
    def get_target(self) -> np.ndarray:
        self._i += 1
        if self._i % self._reset_freq == 0:
            self._target = self._sample_target()
        return self._target