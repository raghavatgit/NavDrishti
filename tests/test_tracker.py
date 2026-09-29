import unittest
import numpy as np
from core.kalman_trajectory_tracker import KalmanObstacleTracker

class TestKalmanTracker(unittest.TestCase):
    def test_tracking_convergence(self):
        tracker = KalmanObstacleTracker(dt=0.1)
        for i in range(10):
            tracker.predict()
            tracker.update(np.array([float(i), float(i * 2)]))
        self.assertAlmostEqual(tracker.state[2], 10.0, delta=1.5)

if __name__ == '__main__':
    unittest.main()
