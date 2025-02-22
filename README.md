# CMPS12 i2C - CircuitPython Library

## General Description

This is a CircuitPython library for interacting with the **[CMPS12](https://www.robot-electronics.co.uk/cmps12-tilt-compensated-magnetic-compass.html)** sensor, a tilt-compensated digital compass that utilizes a **3-axis magnetometer**, **3-axis gyroscope**, and **3-axis accelerometer**. The library allows obtaining **orientation readings (Bearing, pitch, roll), temperature, raw sensor data, and calibration status** from the CMPS12 sensor via I2C communication. Supports communication with the upgraded version of the sensor, the CMPS14.

## Requirements

- **A CircuitPython-compatible development board**, such as the **Raspberry Pi Pico W**.
- **CircuitPython installed on the board**. Follow the installation guide at [CircuitPython.org](https://circuitpython.org/).
- **Additional libraries:**
  - `adafruit_bus_device`

## Installation

1. **Set up CircuitPython on the Raspberry Pi Pico W:**
   - Download the CircuitPython version compatible with the **Raspberry Pi Pico W** from [CircuitPython.org](https://circuitpython.org/board/raspberry_pi_pico_w/).
   - Copy the UF2 file onto the board.

2. **Install the CMPS12 library:**
   - Copy the `cmps_i2c.py` file into the `lib/` folder on your CircuitPython board.

3. **Install `adafruit_bus_device` if it is not already on your device:**
   - Download the library from the [Adafruit bundle](https://circuitpython.org/libraries) and copy `adafruit_bus_device/` into `lib/`.

4. **Main code file**
    - Copy or replace the `code.py` file into the `CIRCUITPY` or  `root` folder on your CircuitPython board.

## Usage

The example file `code.py` demonstrates how to use the library to read data from the CMPS12 sensor.

```python
import board
import busio
from cmps_i2c import CMPS12

# Configure the I2C bus
i2c = busio.I2C(board.GP15, board.GP14)

# Initialize the CMPS12 sensor
sensor = CMPS12(i2c)
if not sensor.begin():
    print("Error: Failed to initialize the sensor")
else:
    print("CMPS12 sensor initialized correctly")

while True:
    # Read data
    bearing = sensor.get_bearing_360()
    pitch = sensor.get_pitch_90()
    roll = sensor.get_roll_90()
    cal_status = sensor.get_cal_stat()
    print(f"Bearing: {bearing}°, Pitch: {pitch}°, Roll: {roll}°, Calibration: {cal_status}")
```

## Library Methods

### `begin()`

**Description:** Initializes the sensor and checks if it responds correctly.

- **Return:** `True` if the sensor initializes correctly, `False` otherwise.

### `get_bearing_360()`

**Description:** Returns the orientation angle (heading) in degrees, ranging from **0-359.9°**.

- **Return:** `float` with the angle in degrees.

### `get_pitch_90()`

**Description:** Returns the pitch angle in degrees within the range **-90° to 90°**.

- **Return:** `int` with the inclination angle.

### `get_pitch_180()`

**Description:** Returns the pitch angle in degrees within the range **-180° to 180°**.

- **Return:** `int` with the inclination angle.

### `get_roll_90()`

**Description:** Returns the roll angle in degrees within the range **-90° to 90°**.

- **Return:** `int` with the roll angle.

### `get_mag_raw()`

**Description:** Returns the raw magnetometer readings for the X, Y, and Z axes.

- **Return:** `tuple (x, y, z)` with **16-bit signed** magnetometer values.

### `get_acc_raw()`

**Description:** Returns the raw accelerometer readings for the X, Y, and Z axes.

- **Return:** `tuple (x, y, z)` with **16-bit signed** accelerometer values.

### `get_gyro_raw()`

**Description:** Returns the raw gyroscope readings for the X, Y, and Z axes.

- **Return:** `tuple (x, y, z)` with **16-bit signed** gyroscope values.

### `get_temp()`

**Description:** Returns the temperature in degrees Celsius.

- **Return:** `int` with the temperature in °C.

### `get_cal_stat()`

**Description:** Returns the sensor's calibration status.

- **Return:** `CalStat.CAL_LOW = 0`, `CalStat.CAL_MED = 2`, or `CalStat.CAL_HIGH = 1`.

### `store_cal()`

**Description:** Stores the current calibration in the CMPS12 memory.

- **Return:** `True` if the storage is successful.

### `erase_cal()`

**Description:** Erases the stored calibration from the sensor.

- **Return:** `True` if the operation is successful.

## Credits

- **Author:** Julian Andres Castro Pardo
- **Contributors:** Based on the manufacturer documentation of **CMPS12**.

## License

This project is licensed under the **MIT License**.
