# RCTR Robot Control System

[![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros\&logoColor=white)](https://docs.ros.org/en/humble/index.html) [![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

A modular robot‑control stack that combines an STM32 micro‑controller running Mbed OS, a ROS 2 control layer, and a YOLOv8‑based perception pipeline. Designed for low‑cost edge hardware and educational robotics projects.

---

## Features

* **Real‑time firmware** for motor and sensor control on an ST NUCLEO‑F091RC board
* **ROS 2 Humble** nodes for hardware abstraction, motion control, and visualization
* **YOLOv8 nano** object‑detection pipeline (training & inference scripts included)
* **Plug‑and‑play USB‑UART bridge** between STM32 and host PC
* **Emergency‑stop topic** for safety‑critical shutdown

## Hardware

| Component          | Tested Version      |
| ------------------ | ------------------- |
| MCU                | ST NUCLEO‑F091RC    |
| USB ↔︎ UART bridge | N/A |
| Host PC            | Rasberry pi    |

>  If you use a different Nucleo board, adjust the pin mappings in `stm32_firmware/mbed_app.json`.

## Software Requirements

* **Python ≥ 3.9**
* **ROS 2 Humble** desktop install
* **Mbed CLI 2** (`pip install mbed‑cli`)
* **CMake ≥ 3.22** (bundled with ROS 2 packages)

---

## Quick‑Start

```bash
# Clone repository
git clone https://github.com/<your‑org>/rctr_robot.git
cd rctr_robot
```

### 1  Compile & Flash STM32 Firmware

```bash
cd stm32_firmware
mbed deploy                               # pulls Mbed OS sources
mbed compile -t GCC_ARM -m NUCLEO_F091RC  # generates ./BUILD/NUCLEO_F091RC/GCC_ARM/*.bin
# Flash: drag‑and‑drop the .bin onto the Nucleo mass‑storage device or use ST‑Link CLI
```

### 2  Set‑up ROS 2 Workspace

```bash
mkdir -p ~/rctr_ws/src
ln -s $(pwd) ~/rctr_ws/src/rctr_robot     # symlink repo into workspace
cd ~/rctr_ws
rosdep install --from-paths src --ignore-src -r -y
pip install -r src/rctr_robot/requirements.txt   # ultralytics, opencv‑python, pyserial
colcon build --packages-select rctr_robot
source install/setup.bash
```

### 3  Train (or Fine‑tune) YOLOv8 Model *optional*

```python
from ultralytics import YOLO

model = YOLO("yolov8n.pt")
model.train(
    data="datasets/rctr/dataset.yaml",
    epochs=100,
    imgsz=640,
    batch=16,
    name="rctr_yolo",
)
model.export(format="onnx", imgsz=640)  # use ONNX for GPU‑less inference if needed
```

> Place the exported `best.onnx` (or `.pt`) file in `rctr_robot/models/` before launching.

### 4  Run the System

```bash
# Terminal 1 – launch entire stack
ros2 launch rctr_robot bringup.launch.py

# Terminal 2 – monitor robot state\ nros2 topic echo /robot_state

# Terminal 3 – view camera & detections
ros2 run rqt_image_view rqt_image_view /perception_viz

# Terminal 4 – emergency stop
ros2 topic pub /hardware_command std_msgs/String "data: STOP"
```

---

## Directory Layout

```
rctr_robot/
├── stm32_firmware/     # Mbed OS application
├── launch/             # ROS 2 launch files
├── rctr_robot/         # Python package (nodes & utilities)
├── models/             # Pre‑trained or fine‑tuned YOLO weights
└── README.md
```

## Contributing

Pull requests are welcome! Please run `pre‑commit run --all-files` before submitting to keep formatting consistent.

## License

This project is licensed under the MIT License – see the [LICENSE](LICENSE) file for details.
