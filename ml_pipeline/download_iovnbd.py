"""
IO-VNBD Dataset Downloader for NAVDRISHTI
Downloads verified empirical driving runs directly from the official GitHub repository:
https://github.com/onyekpeu/IO-VNBD
"""

import os
import sys
import urllib.request

# Ensure UTF-8 output on Windows console
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

REPO_BASE = "https://media.githubusercontent.com/media/onyekpeu/IO-VNBD/master"
DATA_DIR = os.path.join(os.path.dirname(__file__), "data")

AVAILABLE_TRACKS = {
    "S1": {
        "description": "Coventry Ring Road (A4053), B4101, 9 roundabouts, hilly terrain, hard braking",
        "distance_km": 38.16,
        "duration_min": 86.3,
        "s_path": "Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S1/S-S1.csv",
        "v_path": "Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S1/V-S1.csv"
    },
    "Vw1": {
        "description": "Stationary calibration run for IMU sensor bias estimation (Zero-Velocity)",
        "distance_km": 0.0,
        "duration_min": 34.1,
        "s_path": "Synchronised V abd S datasets/Categorised IOVNB Dataset/Vw (Driver E)/Vw01/S-Vw1.csv",
        "v_path": "Synchronised V abd S datasets/Categorised IOVNB Dataset/Vw (Driver E)/Vw01/V-Vw1.csv"
    },
    "Vw13": {
        "description": "M5 Motorway high-speed straight-line travel (94 - 115 km/h)",
        "distance_km": 0.82,
        "duration_min": 0.5,
        "s_path": "Synchronised V abd S datasets/Categorised IOVNB Dataset/Vw (Driver E)/Vw13/S-Vw13.csv",
        "v_path": "Synchronised V abd S datasets/Categorised IOVNB Dataset/Vw (Driver E)/Vw13/V-Vw13.csv"
    }
}

def download_file(url, target_path):
    os.makedirs(os.path.dirname(target_path), exist_ok=True)
    if os.path.exists(target_path) and os.path.getsize(target_path) > 1000:
        print(f"  ✓ Already exists: {os.path.basename(target_path)} ({os.path.getsize(target_path):,} bytes)")
        return target_path

    print(f"  ⬇ Downloading {os.path.basename(target_path)}...")
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    with urllib.request.urlopen(req) as resp, open(target_path, "wb") as out:
        total_size = int(resp.headers.get("content-length", 0))
        downloaded = 0
        chunk_size = 1024 * 64
        while True:
            chunk = resp.read(chunk_size)
            if not chunk:
                break
            out.write(chunk)
            downloaded += len(chunk)
            if total_size > 0:
                percent = downloaded * 100 / total_size
                print(f"\r    Progress: {downloaded:,} / {total_size:,} bytes ({percent:.1f}%)", end="", flush=True)
            else:
                print(f"\r    Downloaded: {downloaded:,} bytes", end="", flush=True)
        print()
    print(f"  ✓ Saved to: {target_path} ({os.path.getsize(target_path):,} bytes)")
    return target_path

def download_track(track_id="S1"):
    if track_id not in AVAILABLE_TRACKS:
        print(f"Unknown track '{track_id}'. Available: {list(AVAILABLE_TRACKS.keys())}")
        return None, None

    info = AVAILABLE_TRACKS[track_id]
    print(f"\n🚘 Fetching IO-VNBD Empirical Track [{track_id}]:")
    print(f"   Route: {info['description']}")
    print(f"   Distance: {info['distance_km']} km | Duration: {info['duration_min']} min")

    s_url = f"{REPO_BASE}/{urllib.request.quote(info['s_path'])}"
    v_url = f"{REPO_BASE}/{urllib.request.quote(info['v_path'])}"

    s_target = os.path.join(DATA_DIR, f"{track_id}_smartphone_imu.csv")
    v_target = os.path.join(DATA_DIR, f"{track_id}_vehicle_obd.csv")

    download_file(s_url, s_target)
    download_file(v_url, v_target)

    return s_target, v_target

if __name__ == "__main__":
    track = sys.argv[1] if len(sys.argv) > 1 else "Vw13" # Start with Vw13 (fast, ~300 lines) or S1
    print("=" * 65)
    print("🛰️ NAVDRISHTI - IO-VNBD Benchmark Dataset Ingestion Engine")
    print("=" * 65)
    download_track(track)
