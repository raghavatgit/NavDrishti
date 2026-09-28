import numpy as np

class ErrorStateKalmanFilter:
    def __init__(self):
        self.state = np.zeros(15) # pos (3), vel (3), att (3), acc_bias (3), gyro_bias (3)
        self.cov = np.eye(15) * 0.01

    def predict(self, dt, imu_acc, imu_gyro):
        pass
