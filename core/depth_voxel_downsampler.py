import numpy as np

class DepthVoxelDownsampler:
    def __init__(self, voxel_size=0.05):
        self.voxel_size = voxel_size

    def filter(self, points: np.ndarray) -> np.ndarray:
        if len(points) == 0:
            return points
        coords = np.floor(points / self.voxel_size).astype(np.int32)
        unique_coords, indices = np.unique(coords, axis=0, return_index=True)
        return points[indices]
