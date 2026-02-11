# hmc5883l_compass

ROS 2 Jazzy driver for the Honeywell HMC5883L 3-axis digital compass (magnetometer) over I2C.

## Features

- Publishes `sensor_msgs/MagneticField` on the `mag/data` topic
- **Fake mode** — generates random magnetic field data without physical hardware
- Calibrate and reset services for hard-iron bias correction
- Runtime `publish_rate` change via `ros2 param set`
- Configurable gain, data output rate, and samples averaged
- Output in SI units (Tesla)

## Prerequisites

- ROS 2 Jazzy
- Python 3
- `smbus2` (only required when `fake_mode` is `false`)

```bash
pip3 install smbus2
```

## Installation

```bash
cd ~/ros2_ws
colcon build --packages-select hmc5883l_compass
source install/setup.bash
```

## Usage

### Launch with default parameters (fake mode)

```bash
ros2 launch hmc5883l_compass hmc5883l_launch.py
```

### Run with real hardware

```bash
ros2 run hmc5883l_compass hmc5883l_node.py --ros-args -p fake_mode:=false
```

### Verify output

```bash
ros2 topic echo /mag/data
```

### Services

```bash
ros2 service call /mag/calibrate std_srvs/srv/Trigger
ros2 service call /mag/reset std_srvs/srv/Trigger
```

### Change publish rate at runtime

```bash
ros2 param set /hmc5883l_compass_node publish_rate 30.0
```

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `fake_mode` | bool | `true` | `true`: generate random data, `false`: read from real I2C device |
| `i2c_bus` | int | `1` | I2C bus number (`/dev/i2c-N`) |
| `device_address` | int | `0x1E` | HMC5883L I2C address (fixed `0x1E`) |
| `publish_rate` | double | `15.0` | Publishing rate in Hz (runtime changeable) |
| `frame_id` | string | `mag_link` | TF frame ID in message headers |
| `gain` | int | `1` | Gain: `0`=±0.88Ga, `1`=±1.3Ga, `2`=±1.9Ga, `3`=±2.5Ga, `4`=±4.0Ga, `5`=±4.7Ga, `6`=±5.6Ga, `7`=±8.1Ga |
| `data_rate` | int | `4` | Data output rate: `0`=0.75Hz, `1`=1.5Hz, `2`=3Hz, `3`=7.5Hz, `4`=15Hz, `5`=30Hz, `6`=75Hz |
| `samples_averaged` | int | `3` | Samples averaged: `0`=1, `1`=2, `2`=4, `3`=8 |
| `magnetic_field_covariance` | double | `0.0` | Diagonal magnetic field covariance (T²), 0 = unknown |

## Package Structure

```
hmc5883l_compass/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── hmc5883l_params.yaml
├── launch/
│   └── hmc5883l_launch.py
├── hmc5883l_compass/
│   ├── __init__.py
│   └── hmc5883l_driver.py
└── nodes/
    └── hmc5883l_node.py
```

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
