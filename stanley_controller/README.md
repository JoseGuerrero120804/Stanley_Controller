# 🐍 `stanley_controller/` — The Python Source

This folder holds all of the package's code. It is split into **ROS 2 nodes** 🤖 that talk to the robot, **offline CLI tools** 🛠️ that work on CSV files, and **pure-Python libraries** 📚 with no ROS dependency, which keeps them easy to unit test.

```
                ┌────────────────────┐
  waypoints ──▶ │   stanley_node 🧭  │ ──▶ Twist / Vector3Stamped / MotorCommands
  pose/speed ─▶ │  ┌──────────────┐  │
                │  │ stanley.py   │  │  lateral  δ
                │  │ speed_ctrl.py│  │  longitudinal u
                │  └──────────────┘  │
                └────────────────────┘
                 uses: path_utils · geometry · mocap_utils
```

---

## 🤖 ROS 2 Nodes

### 🧭 `stanley_node.py` → `stanley_controller`
The main node. On every tick (`control_rate`, default 20 Hz) it:
1. 📍 takes the latest pose (`Odometry` or mocap `PoseStamped`, optionally Y-up) and speed (odometry twist or the `/qcar/velocity` tachometer),
2. 🧭 asks `StanleyController` for the steering angle `δ`,
3. 🐢 chooses `v_ref` (straight or curve speed) from the nearest segment's curvature,
4. 🎚️ asks `SpeedController` for the command `u`,
5. 📤 publishes according to `command_mode`: `twist` | `qcar_sim` / `user_command` | `motor_commands`,
6. 📊 appends a row to the control log (written at the goal or on shutdown).

🛡️ It stops the car on a stale pose or speed, at the goal and on Ctrl+C/SIGTERM. It warns when another publisher shares its command topic. Most gains can be changed live with `ros2 param set`.

### 📝 `path_recorder.py` → `path_recorder`
Subscribes to `Odometry` and saves a waypoint every `min_distance` metres, projected to the front axle if `record_front_axle` is set. A jump larger than `max_jump_distance` (for example a sim reset 💥) stops recording so the path is not corrupted. **Ctrl+C writes the CSV.**

### 🧮 `dead_reckoning.py` → `dead_reckoning`
Pose estimation for the real QCar when there is no motion capture:
- `heading_source: steering` → bicycle model `θ̇ = v·tan(δ)/L`, with `δ` taken from the steering commands 🚲
- `heading_source: imu` → integrates the gyro z-rate with the trapezoidal rule, after estimating its bias while the car stands still 🌀

It starts from the first row of the waypoints file and publishes `Odometry`. When its inputs stop arriving it **stops publishing**, so the controller sees a stale pose and stops the car instead of trusting a frozen pose 🧊.

### 🎮 `qcar_teleop_bridge.py` → `qcar_teleop_bridge`
Converts `teleop_twist_keyboard`'s `/cmd_vel` (`linear.x` → speed, `angular.z` → steering angle, both clamped) into `/qcar_sim/user_command`, and republishes it at a fixed rate with a fresh timestamp so the QCar sim's 1 s command timeout never fires ⏲️.

---

## 🛠️ Offline Tools

### 🔄 `mocap_to_waypoints.py` → `mocap_to_waypoints`
OptiTrack Motive CSV export ➜ Stanley waypoints file + preview PNG. It estimates the rigid body's **yaw offset** from the straight parts of the run, drops the frames after the car stops, projects positions to the front axle and adds `ref_x, ref_y` for dead reckoning.

### 📈 `evaluate_tracking.py` → `evaluate_tracking`
Compares the reference path with the trajectory the car actually drove, using point-to-segment cross-track error. It reports mean and max error, overall and split into **straights vs. curves**, and saves:
- 🗺️ `stanley_comparison.png`: overlay of the reference and executed paths
- 📉 `stanley_signals.png`: `v_ref`/`v`, `u`, `δ`, `e_ct`, `θₑ` over time (from the control log)

Run `ros2 run stanley_controller evaluate_tracking --help` for all options.

---

## 📚 Libraries (ROS-free)

| File | What's inside |
|---|---|
| 🧭 `stanley.py` | `StanleyController` + `StanleyOutput` dataclass. Windowed nearest-segment search, signed `e_ct` (+ = left), saturation, end-of-path detection |
| 🎚️ `speed_controller.py` | `SpeedController` discrete PID: derivative on measurement + low-pass filter, integral clamp + conditional integration, optional feed-forward |
| 🛤️ `path_utils.py` | `load_path()` (with optional front-axle shift), `path_curvatures()`, `classify_segments()` for curve vs. straight |
| 📐 `geometry.py` | `yaw_from_quaternion()`, `normalize_angle()`, `front_axle_position()` |
| 🎥 `mocap_utils.py` | Motive CSV parser (`load_motive_csv`) and Y-up → planar conversions (`x = X, y = −Z`) that stay valid past ±90° headings |
| 📦 `__init__.py` | Marks the Python package (empty) |

---

## 🧭 Conventions Cheat Sheet

- 📏 Units: metres, seconds, radians
- ↺ `δ > 0` steers **left** (REP‑103). Use `steering_sign: -1.0` for vehicles that use the opposite convention
- ⬅️ `e_ct > 0` means the front axle is **left** of the path
- 🔧 Physical steering = command + `steering_trim`, so the controller subtracts the trim before sending
