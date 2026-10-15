#!/usr/bin/env python3
"""
Convert an OptiTrack Motive take (CSV export) into a Stanley waypoints file.

The output has the same columns as path_recorder (x, y, yaw, linear_velocity,
angular_velocity) with x, y the FRONT-axle positions in the planar frame,
plus ref_x, ref_y (tracked rigid-body point) used to initialise dead reckoning.
"""
import argparse
import csv
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from stanley_controller.geometry import front_axle_position, normalize_angle  # noqa: E402
from stanley_controller.mocap_utils import load_motive_csv  # noqa: E402


def estimate_yaw_offset(t, x, y, yaw, min_speed=0.2, max_yaw_rate=math.radians(3.0),
                        half_window=6):
    """
    Estimate the rigid-body heading bias from straight driving.

    On straights the car moves along its forward axis, so the mean of
    (direction of motion - measured yaw) is the correction to add to yaw.
    Returns None if the take has no straight driving.
    """
    unwrapped = np.unwrap(yaw)
    diffs = []
    for i in range(half_window, len(t) - half_window):
        a, b = i - half_window, i + half_window
        dt = t[b] - t[a]
        dist = math.hypot(x[b] - x[a], y[b] - y[a])
        if dt <= 0.0 or dist / dt < min_speed:
            continue
        if abs(unwrapped[b] - unwrapped[a]) / dt > max_yaw_rate:
            continue
        motion = math.atan2(y[b] - y[a], x[b] - x[a])
        diffs.append(normalize_angle(motion - yaw[i]))
    if not diffs:
        return None
    return math.atan2(np.mean(np.sin(diffs)), np.mean(np.cos(diffs)))


def main():
    home = os.path.expanduser('~')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', default=os.path.join(home, 'Qcar_Red_Take 001.csv'),
                        help='Motive CSV export of the reference run.')
    parser.add_argument('--output', default=os.path.join(home, 'qcar_real_waypoints.csv'),
                        help='Waypoints CSV for the stanley_controller node.')
    parser.add_argument('--plot', default=os.path.join(home, 'qcar_real_waypoints.png'),
                        help='Preview plot of the converted path ("" to skip).')
    parser.add_argument('--rigid-body', default='',
                        help='Rigid body name (default: first one in the file).')
    parser.add_argument('--min-distance', type=float, default=0.15,
                        help='Minimum spacing between waypoints (m).')
    parser.add_argument('--front-axle-offset', type=float, default=0.129,
                        help='Distance from the rigid-body pivot to the front axle (m).')
    parser.add_argument('--min-speed', type=float, default=0.05,
                        help='Frames slower than this (m/s) after the car stops are dropped.')
    parser.add_argument('--yaw-offset-deg', type=float, default=None,
                        help='Heading correction added to the rigid-body yaw (deg). '
                             'Default: estimated from the straight parts of the take.')
    args = parser.parse_args()

    take = load_motive_csv(args.input, args.rigid_body)
    t = np.array(take.time)
    x = np.array(take.x)
    y = np.array(take.y)
    yaw = np.array(take.yaw)

    if args.yaw_offset_deg is None:
        yaw_offset = estimate_yaw_offset(t, x, y, yaw)
        if yaw_offset is None:
            yaw_offset = 0.0
            print('No straight driving found, using yaw offset 0.')
    else:
        yaw_offset = math.radians(args.yaw_offset_deg)
    yaw = np.array([normalize_angle(a + yaw_offset) for a in yaw])

    speed = np.hypot(np.gradient(x, t), np.gradient(y, t))
    moving = np.nonzero(speed > args.min_speed)[0]
    if len(moving) == 0:
        raise SystemExit('The rigid body never moves faster than --min-speed.')
    last = moving[-1]
    yaw_rate = np.gradient(np.unwrap(yaw), t)

    rows = []
    last_front = None
    for i in range(0, last + 1):
        fx, fy = front_axle_position(x[i], y[i], yaw[i], args.front_axle_offset)
        if last_front is not None and \
                math.hypot(fx - last_front[0], fy - last_front[1]) < args.min_distance:
            continue
        rows.append((fx, fy, yaw[i], speed[i], yaw_rate[i], x[i], y[i], t[i]))
        last_front = (fx, fy)

    out_dir = os.path.dirname(args.output)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.output, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x', 'y', 'yaw', 'linear_velocity', 'angular_velocity',
                         'ref_x', 'ref_y', 't'])
        writer.writerows(rows)

    path_length = sum(math.hypot(rows[i + 1][0] - rows[i][0], rows[i + 1][1] - rows[i][1])
                      for i in range(len(rows) - 1))
    print(f'Rigid body          : {take.rigid_body} ({len(t)} tracked frames, '
          f'{t[-1] - t[0]:.1f} s)')
    print(f'Yaw offset applied  : {math.degrees(yaw_offset):+.2f} deg '
          f'(use yaw_offset: {yaw_offset:.4f} for the live mocap pose)')
    print(f'Driving speed       : median {np.median(speed[moving]):.3f} m/s, '
          f'max {speed[moving].max():.3f} m/s')
    print(f'Waypoints           : {len(rows)} (front axle, spacing >= {args.min_distance} m), '
          f'path length {path_length:.2f} m')
    print(f'Start pose (pivot)  : x={x[0]:.3f} m, y={y[0]:.3f} m, '
          f'yaw={math.degrees(yaw[0]):.1f} deg')
    print(f'Saved {args.output}')

    if args.plot:
        wp = np.array(rows)
        fig, ax = plt.subplots(figsize=(9, 9))
        ax.plot(x[:last + 1], y[:last + 1], color='0.6', linewidth=1,
                label='Rigid-body pivot (raw)')
        ax.plot(wp[:, 0], wp[:, 1], 'b.-', label='Front-axle waypoints')
        step = max(1, len(wp) // 25)
        ax.quiver(wp[::step, 0], wp[::step, 1], np.cos(wp[::step, 2]), np.sin(wp[::step, 2]),
                  color='r', width=0.003, label='Heading')
        ax.plot(wp[0, 0], wp[0, 1], 'go', markersize=10, label='Start')
        ax.plot(wp[-1, 0], wp[-1, 1], 'rs', markersize=10, label='End')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]  (= -Z in Motive)')
        ax.set_title(f'{os.path.basename(args.input)} -> Stanley waypoints')
        ax.axis('equal')
        ax.grid(True)
        ax.legend()
        fig.savefig(args.plot, dpi=200)
        print(f'Saved {args.plot}')


if __name__ == '__main__':
    main()
