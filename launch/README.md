# 🚀 `launch/` — Launch Files

Every launch file accepts **`params_file:=/path/to/your.yaml`** to override its default config ⚙️. Launch files are grouped by workflow step and by platform.

```
 1️⃣ Teach            2️⃣ Repeat                   3️⃣ Grade (record what really happened)
 record_path ──────▶ stanley_controller ───────▶ record_actual_trajectory
 qcar_record_path ─▶ qcar_stanley_controller ──▶ qcar_record_actual_trajectory
 (mocap_to_wp) ────▶ qcar_real_stanley
 (bag → csv) ──────▶ qcar_imu_stanley ─────────▶ qcar_imu_record_actual_trajectory
```

---

## 🚙 Prius (Gazebo)

| File | Starts | Default config | Extra args |
|---|---|---|---|
| 📝 `record_path.launch.py` | `path_recorder` | `path_recorder_params.yaml` | — |
| 🧭 `stanley_controller.launch.py` | Prius sim (`prius_bringup`) + `stanley_controller` after a delay | `stanley_params.yaml` | `launch_sim` (true), `sim_startup_delay` (5.0 s) |
| 🎯 `record_actual_trajectory.launch.py` | `path_recorder` (executed path) | `actual_trajectory_recorder_params.yaml` | — |

## 🚗 QCar (Gazebo)

| File | Starts | Default config | Extra args |
|---|---|---|---|
| 📝 `qcar_record_path.launch.py` | `path_recorder` + `qcar_teleop_bridge` 🎮 | `qcar_path_recorder_params.yaml` | `keyboard_bridge` (true; set false for a joystick 🕹️) |
| 🧭 `qcar_stanley_controller.launch.py` | QCar sim (`qcar_gazebo`) + `stanley_controller` after a delay | `qcar_stanley_params.yaml` | `launch_sim` (true), `world` (`test_world.sdf`), `sim_startup_delay` (10.0 s) |
| 🎯 `qcar_record_actual_trajectory.launch.py` | `path_recorder` (executed path) | `qcar_actual_trajectory_recorder_params.yaml` | — |

> ⚠️ Use the **same `world`** for tracking that you used when recording the path, and stop the teleop bridge before tracking, or it will override Stanley's commands.

## 🏁 Physical QCar 2

| File | Starts | Default config | Extra args |
|---|---|---|---|
| 🎥 `qcar_real_stanley.launch.py` | `stanley_controller` (+ `dead_reckoning` when needed) | `qcar_real_<pose_source>_params.yaml` | `pose_source`: `mocap` (default) or `dead_reckoning` |
| 🌀 `qcar_imu_stanley.launch.py` | `dead_reckoning` (named `imu_odometry`) + `stanley_controller` | `qcar_imu_params.yaml` | — |
| 🎯 `qcar_imu_record_actual_trajectory.launch.py` | `path_recorder` on `/qcar/imu_odom` | `qcar_imu_actual_trajectory_recorder_params.yaml` | — |

---

## 💡 Examples

```bash
# Prius tracking without relaunching Gazebo
ros2 launch stanley_controller stanley_controller.launch.py launch_sim:=false

# QCar sim in another world, with more time for a slow machine 🐌
ros2 launch stanley_controller qcar_stanley_controller.launch.py world:=my_track.sdf sim_startup_delay:=20.0

# Real car without cameras 🧮
ros2 launch stanley_controller qcar_real_stanley.launch.py pose_source:=dead_reckoning

# Your own tuned parameters 🎛️
ros2 launch stanley_controller qcar_real_stanley.launch.py params_file:=$HOME/my_params.yaml
```

🛑 The `path_recorder` launches save their CSV when you press **Ctrl+C**.
