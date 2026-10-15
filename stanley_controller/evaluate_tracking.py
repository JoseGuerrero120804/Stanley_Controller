#!/usr/bin/env python3
import argparse
import csv
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from stanley_controller.path_utils import classify_segments  # noqa: E402


def load_columns(csv_file, columns):
    with open(csv_file, newline='') as f:
        rows = list(csv.DictReader(f))
    return np.array([[float(row[c]) for c in columns] for row in rows])


def load_xy(csv_file):
    """Load x, y from a waypoints/trajectory CSV, or x_front, y_front from a control log."""
    with open(csv_file, newline='') as f:
        header = next(csv.reader(f))
    if 'x' in header:
        return load_columns(csv_file, ['x', 'y'])
    return load_columns(csv_file, ['x_front', 'y_front'])


def cross_track_errors(actual, reference):
    """
    Distance from each actual point to the closest reference segment.

    Uses point-to-segment distance (not just the nearest vertex) and also
    returns the index of that nearest segment.
    """
    a = reference[:-1]
    b = reference[1:]
    ab = b - a
    ab_len2 = np.sum(ab ** 2, axis=1)
    ab_len2_safe = np.where(ab_len2 == 0, 1.0, ab_len2)

    errors = np.empty(len(actual))
    nearest_segment = np.empty(len(actual), dtype=int)
    for i, p in enumerate(actual):
        t = np.clip(np.sum((p - a) * ab, axis=1) / ab_len2_safe, 0.0, 1.0)
        dists = np.linalg.norm(p - (a + t[:, None] * ab), axis=1)
        j = int(np.argmin(dists))
        errors[i] = dists[j]
        nearest_segment[i] = j
    return errors, nearest_segment


def plot_control_log(log_file, output):
    """Plot speed tracking, throttle, steering and errors over time."""
    cols = ['t', 'v_ref', 'v', 'u', 'delta', 'cross_track_error', 'heading_error']
    data = load_columns(log_file, cols)
    t, v_ref, v, u, delta, e_ct, theta_e = data.T

    speed_err = v_ref - v
    print(f'\nControl log: {len(t)} samples over {t[-1]:.1f} s')
    print(f'Mean |v_ref - v|                  : {np.abs(speed_err).mean():.4f} m/s')
    print(f'RMS cross-track error (controller) : {np.sqrt(np.mean(e_ct ** 2)):.4f} m')
    print(f'Max |delta|                        : {np.abs(delta).max():.4f} rad')

    fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    axes[0].plot(t, v_ref, 'k--', label='v_ref')
    axes[0].plot(t, v, 'b-', label='v (odometry)')
    axes[0].set_ylabel('speed [m/s]')
    axes[1].plot(t, u, 'r-', label='u (PID command)')
    axes[1].set_ylabel('u [m/s]')
    axes[2].plot(t, delta, 'g-', label='delta')
    axes[2].set_ylabel('steering [rad]')
    axes[3].plot(t, e_ct, 'm-', label='e_ct [m]')
    axes[3].plot(t, theta_e, 'c-', label='theta_e [rad]')
    axes[3].set_ylabel('error')
    axes[3].set_xlabel('t [s]')
    for ax in axes:
        ax.grid(True)
        ax.legend(loc='upper right')
    fig.suptitle('Stanley + PID control signals')
    fig.tight_layout()
    fig.savefig(output, dpi=200)
    print(f'Control signal plot saved to {output}')


def main():
    home = os.path.expanduser('~')
    parser = argparse.ArgumentParser(
        description='Compare the recorded reference path against an executed trajectory '
                    '(cross-track error overall and curve vs straight, overlay plot), and '
                    'optionally plot the controller log.'
    )
    parser.add_argument('--reference', default=os.path.join(home, 'stanley_waypoints.csv'),
                        help='Reference path CSV (from path_recorder during teleop).')
    parser.add_argument('--actual',
                        default=os.path.join(home, 'stanley_actual_trajectory.csv'),
                        help='Executed trajectory CSV (path_recorder during tracking), '
                             'or the controller log (uses its x_front, y_front).')
    parser.add_argument('--log', default=os.path.join(home, 'stanley_control_log.csv'),
                        help='Controller log CSV (log_file param); skipped if missing.')
    parser.add_argument('--output', default=os.path.join(home, 'stanley_comparison.png'),
                        help='Output path for the overlay plot.')
    parser.add_argument('--log-output', default=os.path.join(home, 'stanley_signals.png'),
                        help='Output path for the control signal plot.')
    parser.add_argument('--curve-curvature-threshold', type=float, default=0.1,
                        help='Curvature threshold (rad/m) to classify a segment as curve.')
    args = parser.parse_args()

    reference = load_xy(args.reference)
    actual = load_xy(args.actual)

    errors, nearest_segment = cross_track_errors(actual, reference)
    seg_is_curve = np.array(classify_segments(reference, args.curve_curvature_threshold))
    point_is_curve = seg_is_curve[nearest_segment]
    curve_errors = errors[point_is_curve]
    straight_errors = errors[~point_is_curve]

    def mean_or_nan(e):
        return e.mean() if len(e) else float('nan')

    print(f'Reference waypoints : {len(reference)}')
    print(f'Actual waypoints    : {len(actual)}')
    print(f'Mean cross-track error (overall)  : {errors.mean():.4f} m')
    print(f'Max cross-track error (overall)   : {errors.max():.4f} m')
    print(f'Mean cross-track error (straights): {mean_or_nan(straight_errors):.4f} m '
          f'over {len(straight_errors)} pts')
    print(f'Mean cross-track error (curves)   : {mean_or_nan(curve_errors):.4f} m '
          f'over {len(curve_errors)} pts')

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.plot(reference[:, 0], reference[:, 1], 'b-', linewidth=2, label='Reference path')
    if seg_is_curve.any():
        curve_pts = reference[:-1][seg_is_curve]
        ax.scatter(curve_pts[:, 0], curve_pts[:, 1], c='orange', s=10, zorder=3,
                   label='Reference curve segments')
    ax.plot(actual[:, 0], actual[:, 1], 'r--', linewidth=2, label='Executed trajectory')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title(f'Stanley Tracking: reference vs executed\n'
                 f'Mean cross-track error: {errors.mean():.3f} m '
                 f'(straight: {mean_or_nan(straight_errors):.3f} m, '
                 f'curve: {mean_or_nan(curve_errors):.3f} m)')
    ax.legend()
    ax.axis('equal')
    ax.grid(True)
    out_dir = os.path.dirname(args.output)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    fig.savefig(args.output, dpi=300)
    print(f'\nOverlay plot saved to {args.output}')

    if args.log and os.path.isfile(args.log):
        plot_control_log(args.log, args.log_output)


if __name__ == '__main__':
    main()
