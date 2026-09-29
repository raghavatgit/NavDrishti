import numpy as np

def compute_ate_rmse(estimated_traj, ground_truth):
    # Absolute Trajectory Error Root Mean Square Error computation
    return np.sqrt(np.mean(np.sum((estimated_traj - ground_truth)**2, axis=1)))
