# 🏷️ `resource/` — ament Index Marker

This folder contains one **empty** file named `stanley_controller` 📭.

## 🤔 Why Does an Empty File Matter?

`setup.py` installs it to:

```
install/stanley_controller/share/ament_index/resource_index/packages/stanley_controller
```

The **ament resource index** 📇 uses the presence of that file to register the package. Without it:

- 🔍 `ros2 pkg list` won't show `stanley_controller`
- 🚫 `ros2 run` / `ros2 launch stanley_controller ...` can't find the package
- 📂 `get_package_share_directory('stanley_controller')`, which every launch file here uses, fails

## ⚠️ Rules

- ✋ Don't rename or delete it. The filename must match the package name in `package.xml` and `setup.py`
- 📭 It stays empty. Only its existence matters
- 📝 This README is **not** installed: `setup.py` copies only `resource/stanley_controller`
