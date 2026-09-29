import numpy as np

class KalmanObstacleTracker:
    def __init__(self, dt=0.033):
        self.dt = dt
        self.state = np.zeros(4) # [x, y, vx, vy]
        self.P = np.eye(4) * 1.0

    def predict(self):
        F = np.array([
            [1, 0, self.dt, 0],
            [0, 1, 0, self.dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        self.state = F @ self.state
        self.P = F @ self.P @ F.T + np.eye(4) * 0.01

    def update(self, measurement: np.ndarray):
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        y = measurement - H @ self.state
        S = H @ self.P @ H.T + np.eye(2) * 0.1
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
