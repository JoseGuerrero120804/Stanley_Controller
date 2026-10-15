# ⚙️ `config/` — Parameter Files

Each YAML holds the tuned parameters for one **platform** 🚙🚗🏁 and one **step** of the workflow (📝 teach / 🧭 repeat / 🎯 grade). They are installed to `share/stanley_controller/config` and loaded by the matching launch file.

> ⚠️ **Edit the paths first!** `path_file`, `output_file` and `log_file` are absolute paths under `/home/nombre_usuario/`. Point them at your own home directory. Setting `log_file: ''` disables the control log.

---

## 🗂️ Which File Is Which?

| File | Platform | Step | Node(s) | Highlights |
|---|---|---|---|---|
| `path_recorder_params.yaml` | 🚙 Prius sim | 📝 | `path_recorder` | `/odom`, 0.2 m spacing, wheelbase 2.7 m |
| `stanley_params.yaml` | 🚙 Prius sim | 🧭 | `stanley_controller` | `Twist` output, 1.5 / 1.0 m/s, \|δ\| ≤ 0.44 rad |
| `actual_trajectory_recorder_params.yaml` | 🚙 Prius sim | 🎯 | `path_recorder` | Same reference point as the path |
| `qcar_path_recorder_params.yaml` | 🚗 QCar sim | 📝 | `path_recorder` | `/qcar_sim/odom`, 0.15 m spacing, front axle 0.1296 m ahead of `base_footprint` |
| `qcar_stanley_params.yaml` | 🚗 QCar sim | 🧭 | `stanley_controller` | `qcar_sim` mode, 0.3 / 0.2 m/s (sim caps at 0.446 m/s) |
| `qcar_actual_trajectory_recorder_params.yaml` | 🚗 QCar sim | 🎯 | `path_recorder` | Matches the QCar path recorder |
| `qcar_real_mocap_params.yaml` | 🏁 Real QCar 2 | 🧭 | `stanley_controller` | Live OptiTrack `PoseStamped`, `pose_y_up: true`, `yaw_offset: -0.0603` |
| `qcar_real_dead_reckoning_params.yaml` | 🏁 Real QCar 2 | 🧭 | `stanley_controller` + `dead_reckoning` | Tachometer + steering bicycle model, no cameras |
| `qcar_imu_params.yaml` | 🏁 Real QCar 2 | 🧭 | `imu_odometry` + `stanley_controller` | Gyro odometry, throttle duty cycle, **very slow** 🐢 (0.12 / 0.10 m/s) |
| `qcar_imu_actual_trajectory_recorder_params.yaml` | 🏁 Real QCar 2 | 🎯 | `path_recorder` | `/qcar/imu_odom`, 0.05 m spacing |

---

## 🎛️ Key `stanley_controller` Knobs

| Group | Parameter | What it does |
|---|---|---|
| 📄 Path | `path_file`, `path_front_axle_offset` | Waypoints CSV; shifts each waypoint forward along its yaw if it stores a rear/reference point |
| 📍 Pose | `pose_source` (`odom` / `pose_stamped`), `odom_topic`, `pose_topic`, `pose_y_up`, `yaw_offset` | Where the pose comes from and how to interpret it |
| 🏎️ Speed | `speed_source` (`odom` / `qcar_velocity`), `speed_topic` | PID feedback source |
| 📤 Output | `command_mode` (`twist` / `qcar_sim` / `user_command` / `motor_commands`), `cmd_vel_topic`, `steering_sign`, `steering_trim` | How commands are published |
| 🧭 Stanley | `stanley_gain` (k), `softening_gain` (k_s), `max_steering_angle`, `goal_tolerance`, `wheelbase`, `front_axle_offset`, `search_window` | Lateral controller |
| 🐢 Speed plan | `target_speed`, `target_speed_curve`, `curve_curvature_threshold` (rad/m) | Slows down in curves |
| 🎚️ PID | `kp`, `ki`, `kd`, `integral_limit`, `derivative_filter_alpha`, `feedforward_gain`, `min_command`, `max_command` | Longitudinal controller + saturation |
| 🛡️ Safety | `odom_timeout`, `control_rate` | Stops the car on stale data |

💡 `front_axle_offset < 0` means "use `wheelbase`", i.e. the odometry reports the rear axle.

🔴 Live tuning works for the Stanley gains, the PID gains, the speed targets, `steering_sign`, `steering_trim` and `yaw_offset`:
```bash
ros2 param set /stanley_controller stanley_gain 0.7
ros2 param set /stanley_controller target_speed_curve 0.15
```

---

## 🧪 Field Notes (from the comments in the YAMLs)

- 🐌 **IMU setup:** the car reacts about 2.2 s after each command, so above ~0.2 m/s Stanley weaves. Keep `kd: 0` and cap the throttle at `max_command: 0.065`.
- 🧘 **IMU startup:** keep the car still for `bias_window` (3 s) while the gyro bias is estimated.
- 🎥 **Mocap:** check `ros2 topic echo <pose_topic>`. If `position.y ≈ 0.17` m (marker height), Motive streams Y-up, so set `pose_y_up: true`.
- 🔁 **`steering_sign`:** set it to `-1.0` if a positive command steers the car right, and keep it the same in `dead_reckoning`.
