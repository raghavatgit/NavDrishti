from typing import List, Dict, Any

def classify_proximity(depth_map, bounding_boxes: List[Dict[str, int]]) -> List[Dict[str, Any]]:
    results = []
    for box in bounding_boxes:
        x, y, w, h = box['x'], box['y'], box['w'], box['h']
        mean_depth = 2.5 # Mock processed depth in meters
        results.append({
            'label': box.get('label', 'obstacle'),
            'distance_m': round(mean_depth, 2),
            'severity': 'HIGH' if mean_depth < 1.2 else 'NORMAL'
        })
    return results
