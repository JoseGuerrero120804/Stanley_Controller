import math
import random

from stanley_controller.mocap_utils import (load_motive_csv, y_up_to_planar,
                                            yaw_from_motive_euler_xyz,
                                            yaw_from_y_up_quaternion)


def angle_close(a, b, tol=1e-6):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b))) < tol


def test_pure_rotation_about_up_axis_is_yaw():
    assert angle_close(yaw_from_motive_euler_xyz(0.0, 30.0, 0.0), math.radians(30.0))


def test_euler_y_column_folding_past_90_degrees():
    # Heading 150 deg is exported by Motive as X = Z = 180, Y = 30.
    assert angle_close(yaw_from_motive_euler_xyz(180.0, 30.0, 180.0), math.radians(150.0))


def test_quaternion_and_euler_agree():
    rng = random.Random(0)
    for _ in range(50):
        heading = rng.uniform(-math.pi, math.pi)
        half = heading / 2.0
        # Rotation about +Y by `heading`.
        q = (0.0, math.sin(half), 0.0, math.cos(half))
        assert angle_close(yaw_from_y_up_quaternion(*q), heading)
        # Same rotation as Motive XYZ Euler: |Y| <= 90 deg, X = Z = 180 beyond that.
        if abs(heading) <= math.pi / 2:
            euler = (0.0, math.degrees(heading), 0.0)
        else:
            euler = (180.0, math.degrees(math.copysign(math.pi, heading) - heading), 180.0)
        assert angle_close(yaw_from_motive_euler_xyz(*euler), heading)


def test_y_up_to_planar_is_right_handed():
    # Moving along Motive -Z is moving along planar +y (left of +x).
    assert y_up_to_planar(1.0, -2.0) == (1.0, 2.0)


def test_load_motive_csv(tmp_path):
    lines = [
        'Format Version,1.23,Rotation Type,XYZ,Length Units,Millimeters',
        '',
        ',Type,Rigid Body,Rigid Body,Rigid Body,Rigid Body,Rigid Body,Rigid Body,Rigid Body',
        ',Name,Car,Car,Car,Car,Car,Car,Car',
        ',ID,1,1,1,1,1,1,1',
        ',,Rotation,Rotation,Rotation,Position,Position,Position,Mean Marker Error',
        'Frame,Time (Seconds),X,Y,Z,X,Y,Z,',
        '0,0.0,0,90,0,1000,170,-500,0.1',
        '1,0.1,,,,,,,',
        '2,0.2,0,90,0,1000,170,-600,0.1',
    ]
    f = tmp_path / 'take.csv'
    f.write_text('\n'.join(lines) + '\n')
    take = load_motive_csv(str(f))
    assert take.rigid_body == 'Car'
    assert take.time == [0.0, 0.2]           # untracked frame skipped
    assert math.isclose(take.x[0], 1.0)      # mm -> m
    assert math.isclose(take.y[1], 0.6)      # y = -Z
    assert angle_close(take.yaw[0], math.pi / 2)
