# import time
import board
import busio
import digitalio
from time import sleep, monotonic
from cmps_i2c import CMPS12

# Initialize Board LED
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

# Configure the I2C bus on the Raspberry Pi Pico W
scl_pin = board.GP15 # GP1
sda_pin = board.GP14 # GP0
i2c_bus = busio.I2C(scl_pin, sda_pin)

# Select the sensor type; use CMPS12 or CMPS14 depending on the one you have
sensor = CMPS12(i2c_bus)  # Or use CMPS14(i2c_bus) if you are using the CMPS14

# Initialize the sensor
if not sensor.begin():
    print("Error: Failed to initialize sensor. Please check the connection.")
else:
    print("Sensor initialized correctly.")

# Calibration status
cal_stat = sensor.get_cal_stat() # Is calibrate if value is 1

# Store and delete calibration
if cal_stat == 1:
    sensor.store_cal()
    # print("Almacenando calibración...")
sleep(1)

# print("Borrando calibración...")
# sensor.erase_cal()
# time.sleep(1)

# Main loop
def main():
    start = monotonic()  # Capture current time
    while True:
        # Calibration status
        cal_stat = sensor.get_cal_stat() # Is calibrate if value is 1
        bearing = sensor.get_bearing_360()  # Drift angle (heading, yaw)
        pitch   = sensor.get_pitch_180()    # Inclination angle (cabeceo, pitch)
        roll    = sensor.get_roll_90()         # Heel angle (alabeo, roll)
        
        # Bearing bosch BNO055
        bearing_bosch = sensor.get_bearing_bosch_BNO055()
        
        # Temperature (only in CMPS12)
        if isinstance(sensor, CMPS12):
            temp = sensor.get_temp()
            # print(f"Temperature: {temp} °C")

        time_elapsed = int(monotonic() - start)
        # print(f"B: {bearing}°, P: {pitch}°, R:{roll}°, Temp: {temp}°C, Cal_stat: {cal_stat}, Stopwatch: {time_elapsed}")
        print(f"{bearing},{pitch},{roll},{temp},{cal_stat}")
        # print(f"Bearing Bosch: {bearing_bosch}°\t Cal stat: {cal_stat}")
        sleep(0.1)
        
        led.value = (time_elapsed % 5 == 0)

    
if __name__ == "__main__":
    main()