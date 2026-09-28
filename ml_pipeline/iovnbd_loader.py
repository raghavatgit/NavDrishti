"""
NAVDRISHTI IO-VNBD PyTorch Dataset Loader
Loads and preprocesses real empirical vehicle telemetry from the IO-VNBD benchmark dataset:
- Smartphone MEMS IMU: 3-axis Accelerometer (m/s^2), 3-axis Gyroscope (rad/s)
- Vehicle CAN-bus Ground Truth: Forward Velocity (m/s), Lateral Velocity (0 m/s via NHC), Yaw Rate (rad/s)
"""

import os
import sys
import numpy as np

if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

try:
    import torch
    from torch.utils.data import Dataset, DataLoader
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False
    Dataset = object

DATA_DIR = os.path.join(os.path.dirname(__file__), "data")

class IOVNBDataset(Dataset):
    def __init__(self, track_id="Vw13", window_len=50, stride=5):
        """
        Loads empirical driving run from IO-VNBD.
        Args:
            track_id: ID of the track ('Vw13', 'S1', 'Vw1', etc.)
            window_len: Temporal window length (samples). 50 samples = 0.5s @ 100Hz.
            stride: Sliding window stride.
        """
        self.track_id = track_id
        self.window_len = window_len
        self.stride = stride

        s_file = os.path.join(DATA_DIR, f"{track_id}_smartphone_imu.csv")
        v_file = os.path.join(DATA_DIR, f"{track_id}_vehicle_obd.csv")

        if not os.path.exists(s_file) or not os.path.exists(v_file):
            raise FileNotFoundError(
                f"IO-VNBD track files not found in {DATA_DIR}. "
                f"Please run: python ml_pipeline/download_iovnbd.py {track_id}"
            )

        self.imu_windows, self.targets = self._load_and_process(s_file, v_file)

    def _load_and_process(self, s_file, v_file):
        # 1. Parse Smartphone IMU CSV
        with open(s_file, "r", encoding="utf-8", errors="ignore") as f:
            headers = [h.strip() for h in f.readline().split(",")]
            s_rows = []
            for line in f:
                if line.strip():
                    parts = [p.strip() for p in line.split(",")]
                    if len(parts) >= len(headers):
                        s_rows.append(parts)

        # 2. Parse Vehicle CAN-bus CSV
        with open(v_file, "r", encoding="utf-8", errors="ignore") as f:
            v_headers = [h.strip() for h in f.readline().split(",")]
            v_rows = []
            for line in f:
                if line.strip():
                    parts = [p.strip() for p in line.split(",")]
                    if len(parts) >= len(v_headers):
                        v_rows.append(parts)

        # Extract Smartphone IMU features
        # Columns: ACCELEROMETER X, Y, Z, GYROSCOPE Yaw, Pitch, Roll
        ax_idx = next((i for i, h in enumerate(headers) if "ACCELEROMETER X" in h), 9)
        ay_idx = next((i for i, h in enumerate(headers) if "ACCELEROMETER Y" in h), 10)
        az_idx = next((i for i, h in enumerate(headers) if "ACCELEROMETER Z" in h), 11)
        gy_idx = next((i for i, h in enumerate(headers) if "GYROSCOPE Yaw" in h), 15)
        gp_idx = next((i for i, h in enumerate(headers) if "GYROSCOPE Pitch" in h), 16)
        gr_idx = next((i for i, h in enumerate(headers) if "GYROSCOPE Roll" in h), 17)

        # Extract Vehicle Ground Truth
        # Columns: Velocity (km/hr), Yaw Rate (deg/sec)
        vel_idx = next((i for i, h in enumerate(v_headers) if "Velocity (km/hr)" in h), 4)
        yaw_idx = next((i for i, h in enumerate(v_headers) if "Yaw Rate (deg/sec)" in h), 14)

        n_samples = min(len(s_rows), len(v_rows))
        
        imu_series = []
        target_series = []

        for i in range(n_samples):
            try:
                ax = float(s_rows[i][ax_idx])
                ay = float(s_rows[i][ay_idx])
                az = float(s_rows[i][az_idx])
                gy = float(s_rows[i][gy_idx])
                gp = float(s_rows[i][gp_idx])
                gr = float(s_rows[i][gr_idx])

                v_kmh = float(v_rows[i][vel_idx])
                yaw_deg = float(v_rows[i][yaw_idx])

                v_ms = v_kmh / 3.6
                yaw_rad = yaw_deg * (np.pi / 180.0)

                imu_series.append([ax, ay, az, gy, gp, gr])
                target_series.append([v_ms, 0.0, yaw_rad]) # [v_forward, v_lat=0 (NHC), yaw_rate]
            except (ValueError, IndexError):
                continue

        imu_np = np.array(imu_series, dtype=np.float32) # [N, 6]
        target_np = np.array(target_series, dtype=np.float32) # [N, 3]

        # Form temporal windows
        windows = []
        targets = []
        for start in range(0, len(imu_np) - self.window_len, self.stride):
            end = start + self.window_len
            w = imu_np[start:end, :].T # [6, window_len]
            t = target_np[end - 1, :] # Target at end of window
            windows.append(w)
            targets.append(t)

        return np.array(windows, dtype=np.float32), np.array(targets, dtype=np.float32)

    def __len__(self):
        return len(self.imu_windows)

    def __getitem__(self, idx):
        x = self.imu_windows[idx]
        y = self.targets[idx]
        if TORCH_AVAILABLE:
            return torch.tensor(x), torch.tensor(y)
        return x, y

def get_iovnbd_loader(track_id="Vw13", batch_size=32, window_len=50, shuffle=True):
    dataset = IOVNBDataset(track_id=track_id, window_len=window_len)
    print(f"📊 Loaded IO-VNBD [{track_id}]: {len(dataset)} empirical training windows (window_len={window_len})")
    if TORCH_AVAILABLE:
        return DataLoader(dataset, batch_size=batch_size, shuffle=shuffle)
    return dataset

if __name__ == "__main__":
    loader = get_iovnbd_loader("Vw13", batch_size=16)
    print("✅ IO-VNBD empirical dataset successfully loaded and validated.")
