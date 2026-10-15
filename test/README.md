# 🧪 `test/` — Tests & Linters

Unit tests for the ROS-free core, plus the standard ament style checks ✅.

## ▶️ Run Them

```bash
# The ROS way 🐢
colcon test --packages-select stanley_controller
colcon test-result --verbose

# Or directly with pytest ⚡ (from the package root, after sourcing the workspace)
python3 -m pytest test/ -v
```

---

## 📋 What's Covered

### 🧭🎚️ `test_controllers.py` — Stanley + PID

| Test | Checks |
|---|---|
| `test_on_path_aligned_gives_zero_steering` | On the path and aligned with it → `e_ct = 0`, `δ = 0` 🎯 |
| `test_cross_track_sign_left_positive_and_steers_back` | Left of path → `e_ct > 0`, steers right; and the mirror case ↔️ |
| `test_front_axle_projection_and_heading_error` | Rear → front axle projection and the `θₑ` sign 📐 |
| `test_steering_saturation` | Large error → `δ` clipped to `−max_steering_angle` 🧱 |
| `test_path_end_detected` | Passing the last waypoint sets `at_path_end` 🏁 |
| `test_pid_converges_on_velocity_plant` | PID drives an ideal velocity plant to 1.5 m/s 📈 |
| `test_pid_output_is_saturated` | Output stays within `[u_min, u_max]` 🔒 |
| `test_pid_anti_windup_freezes_integral_in_saturation` | Integral frozen while saturated, recovers immediately 🧯 |
| `test_pid_derivative_on_measurement_has_no_setpoint_kick` | A step in `v_ref` causes no derivative kick 🦵 |

### 🎥 `test_mocap_utils.py` — OptiTrack Helpers

| Test | Checks |
|---|---|
| `test_pure_rotation_about_up_axis_is_yaw` | Rotation about +Y gives the planar yaw 🔄 |
| `test_euler_y_column_folding_past_90_degrees` | Headings past ±90° (Motive's Euler fold) are handled 🌀 |
| `test_quaternion_and_euler_agree` | 50 random headings: quaternion and Euler paths agree 🎲 |
| `test_y_up_to_planar_is_right_handed` | `(X, Z) → (x, y = −Z)` 🤚 |
| `test_load_motive_csv` | Parses a small Motive CSV: mm → m, skips untracked frames, picks the rigid body 📄 |

### 🧹 Linters

| File | Tool |
|---|---|
| `test_flake8.py` | `ament_flake8`, PEP 8 style 🐍 |
| `test_pep257.py` | `ament_pep257`, docstring conventions 📝 |

---

💡 **Tip:** `stanley.py`, `speed_controller.py`, `path_utils.py`, `geometry.py` and `mocap_utils.py` don't import ROS, so the unit tests run in milliseconds and need no simulator 🚀.
