import csv
import math
import os


def load_path(path_file, front_axle_offset=0.0):
    """
    Read the (x, y) columns of a waypoints CSV.

    front_axle_offset > 0 shifts every waypoint forward along its 'yaw' column,
    turning a reference-point trajectory (e.g. rear axle) into the front-axle
    path that Stanley tracks.
    """
    if not os.path.isfile(path_file):
        raise RuntimeError(
            f'Path file {path_file} does not exist. Record the reference path first '
            '(record_path.launch.py / qcar_record_path.launch.py) or set path_file.')
    path = []
    with open(path_file, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            x, y = float(row['x']), float(row['y'])
            if front_axle_offset:
                yaw = float(row['yaw'])
                x += front_axle_offset * math.cos(yaw)
                y += front_axle_offset * math.sin(yaw)
            path.append((x, y))
    if len(path) < 2:
        raise RuntimeError(f'Path file {path_file} must contain at least 2 waypoints')
    return path


def path_curvatures(path):
    """
    Local curvature (rad/m) at each point of the path.

    Heading change between consecutive segments divided by the segment length,
    so the value does not depend on how finely the path was sampled.
    Endpoints (undefined) are 0.0.
    """
    n = len(path)
    curvatures = [0.0] * n
    for i in range(1, n - 1):
        x0, y0 = path[i - 1]
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        h1 = math.atan2(y1 - y0, x1 - x0)
        h2 = math.atan2(y2 - y1, x2 - x1)
        dh = abs(math.atan2(math.sin(h2 - h1), math.cos(h2 - h1)))
        seg_len = math.hypot(x2 - x1, y2 - y1)
        curvatures[i] = dh / seg_len if seg_len > 1e-9 else 0.0
    return curvatures


def classify_segments(path, curvature_threshold):
    """
    Flag which path segments belong to a curve.

    Returns a list of length len(path) - 1: True if segment i -> i+1 has either
    endpoint's local curvature >= curvature_threshold (rad/m).
    """
    point_curvatures = path_curvatures(path)
    return [
        point_curvatures[i] >= curvature_threshold
        or point_curvatures[i + 1] >= curvature_threshold
        for i in range(len(path) - 1)
    ]
