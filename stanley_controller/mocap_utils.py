"""
OptiTrack Motive helpers: CSV take parsing and Y-up -> planar conversion.

Motive uses a right-handed Y-up frame: the floor is the X-Z plane and Y is the
height. The planar (ROS, Z-up) frame used by the controller is

    x = X,   y = -Z,   yaw = rotation about +Y

which is exactly what Motive itself streams when its Up Axis is set to Z-up.
The yaw is taken from the full rotation (forward axis of the rigid body), not
from the Euler 'Y' column alone: with Euler angles the Y angle folds back
once the car turns past +/-90 deg, while X and Z jump to +/-180 deg.
"""
import csv
from dataclasses import dataclass
import math

_UNIT_SCALE = {'millimeters': 0.001, 'centimeters': 0.01, 'meters': 1.0}


@dataclass
class MotiveTake:
    time: list    # s
    x: list       # m, planar frame
    y: list       # m, planar frame
    yaw: list     # rad, planar frame, rigid-body +X axis
    rigid_body: str


def _rotation_matrix_xyz(rx, ry, rz):
    """R = Rx(rx) @ Ry(ry) @ Rz(rz) (Motive 'XYZ' Euler order), angles in rad."""
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return (
        (cy * cz, -cy * sz, sy),
        (sx * sy * cz + cx * sz, -sx * sy * sz + cx * cz, -sx * cy),
        (-cx * sy * cz + sx * sz, cx * sy * sz + sx * cz, cx * cy),
    )


def yaw_from_y_up_forward(fx, fz):
    """Planar yaw of a Y-up forward vector (its X and Z components)."""
    return math.atan2(-fz, fx)


def yaw_from_motive_euler_xyz(rx_deg, ry_deg, rz_deg):
    """Planar yaw of the rigid body's +X axis from Motive XYZ Euler angles (deg)."""
    r = _rotation_matrix_xyz(*(math.radians(a) for a in (rx_deg, ry_deg, rz_deg)))
    return yaw_from_y_up_forward(r[0][0], r[2][0])


def yaw_from_y_up_quaternion(qx, qy, qz, qw):
    """Planar yaw of the rigid body's +X axis from a Y-up quaternion."""
    fx = 1.0 - 2.0 * (qy * qy + qz * qz)
    fz = 2.0 * (qx * qz - qw * qy)
    return yaw_from_y_up_forward(fx, fz)


def y_up_to_planar(x, z):
    """Motive Y-up floor coordinates (X, Z) -> planar (x, y)."""
    return x, -z


def load_motive_csv(csv_file, rigid_body=''):
    """
    Read the pose of one rigid body from a Motive CSV export.

    Frames where the rigid body was not tracked (empty cells) are skipped.
    rigid_body='' selects the first rigid body in the file.
    """
    with open(csv_file, newline='') as f:
        rows = list(csv.reader(f))

    meta = rows[0]
    settings = {meta[i]: meta[i + 1] for i in range(0, len(meta) - 1, 2) if meta[i]}
    units = settings.get('Length Units', 'Meters').lower()
    if units not in _UNIT_SCALE:
        raise RuntimeError(f'Unsupported Motive length units: {units}')
    scale = _UNIT_SCALE[units]
    rotation_type = settings.get('Rotation Type', 'XYZ')

    # Header rows: Type / Name / ID / Rotation-Position / axis, data afterwards.
    type_row = next(i for i, r in enumerate(rows) if len(r) > 1 and r[1] == 'Type')
    types, names = rows[type_row], rows[type_row + 1]
    kinds, axes = rows[type_row + 3], rows[type_row + 4]
    data_start = type_row + 5

    if not rigid_body:
        rigid_body = next(n for t, n in zip(types, names) if t == 'Rigid Body')

    def column(kind, axis):
        for i, (t, n, k, a) in enumerate(zip(types, names, kinds, axes)):
            if t == 'Rigid Body' and n == rigid_body and k == kind and a == axis:
                return i
        raise RuntimeError(f'Column {kind} {axis} of rigid body {rigid_body!r} not found '
                           f'in {csv_file}')

    pos_cols = [column('Position', a) for a in ('X', 'Z')]
    quaternion = rotation_type.lower() == 'quaternion'
    rot_axes = ('X', 'Y', 'Z', 'W') if quaternion else ('X', 'Y', 'Z')
    rot_cols = [column('Rotation', a) for a in rot_axes]
    time_col = 1

    take = MotiveTake([], [], [], [], rigid_body)
    for r in rows[data_start:]:
        try:
            t = float(r[time_col])
            px, pz = (float(r[c]) * scale for c in pos_cols)
            rot = [float(r[c]) for c in rot_cols]
        except (ValueError, IndexError):
            continue
        if quaternion:
            yaw = yaw_from_y_up_quaternion(*rot)
        else:
            yaw = yaw_from_motive_euler_xyz(*rot)
        x, y = y_up_to_planar(px, pz)
        take.time.append(t)
        take.x.append(x)
        take.y.append(y)
        take.yaw.append(yaw)
    if len(take.time) < 2:
        raise RuntimeError(f'No tracked frames for rigid body {rigid_body!r} in {csv_file}')
    return take
