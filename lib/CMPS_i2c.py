# Raspberry Pi Pico W - CMPS12 tilt-compensated digital compass.
# Description: This code reads the data from the CMPS12 and prints it to the serial monitor.
# Author: github.com/JuliansCastro
# License: MIT


import time
from adafruit_bus_device.i2c_device import I2CDevice


class CMPS1x:
    '''Base class for CMPS12 and CMPS14 compasses. Support for CircuitPython 7.0.0 and above. 
    Use CMPS12 or CMPS14 instead.
       
    CMPS12: 16-bit bearing, 8-bit pitch and roll, temperature, and calibration
    CMPS14: 16-bit bearing, 16-bit pitch and roll, and calibration
    
    The CMPS12 and CMPS14 are digital tilt compasses that provide accurate heading
    information using a 3-axis magnetometer, a 3-axis accelerometer, and a
    3-axis gyroscope. They also provide pitch and roll information. 
    
    i2c: The I2C bus the sensor is connected to
    address: The I2C address of the sensor. Default is 0x60 for CMPS12 and CMPS14
            
    '''
    def __init__(self, i2c, address=0x60):
        self.i2c_device = I2CDevice(i2c, address)
        self.address = address

    def _read_register(self, register, num_bytes=1):
        with self.i2c_device as i2c:
            i2c.write(bytes([register]))
            result = bytearray(num_bytes)
            i2c.readinto(result)
        if num_bytes == 1:
            return result[0] # & 0xFF Mask the value to 8-bits (0-255)
        elif num_bytes == 2:
            return result[0] << 8 | result[1] # & 0xFFFF Mask the value to 16-bits (0-65535)
        return result

    def _write_sequence(self, sequence):
        with self.i2c_device as i2c:
            i2c.write(bytes(sequence))
        time.sleep(0.02)

class CalStat():
    '''Calibration status for the CMPS12 and CMPS14 compasses
    CAL_LOW:    Low calibration     - Value = 0
    CAL_HIGH:   High calibration    - Value = 1
    CAL_MED:    Medium calibration  - Value = 2
    '''    
    CAL_LOW     = 0
    CAL_HIGH    = 1
    CAL_MED     = 2

class CMPS12(CMPS1x):
    CMPS12_REG_COMMAND         = 0x00  # Command register / Software version
    CMPS12_REG_BEARING255      = 0x01  # Bearing register (8-bit) i.e 0 to 255 for a full circle
    CMPS12_REG_BEARING16       = 0x02  # Bearing register (16-bit) i.e 0 to 3599 (0-359.9)°. Calculated by processor from Quaternion output of the BNO055
    CMPS12_REG_PITCH90         = 0x04  # Pitch angle - signed byte giving angle in degrees from the horizontal plane (+/- 90°)
    CMPS12_REG_ROLL90          = 0x05  # Roll angle - signed byte giving angle in degrees from the horizontal plane (+/- 90°)
    CMPS12_REG_MAGNRAW_X       = 0x06  # Raw magnetometer X-axis data (16-bit) - signed
    CMPS12_REG_MAGNRAW_Y       = 0x08  # Raw magnetometer Y-axis data (16-bit) - signed
    CMPS12_REG_MAGNRAW_Z       = 0x0A  # Raw magnetometer Z-axis data (16-bit) - signed
    CMPS12_REG_ACCRAW_X        = 0x0C  # Raw accelerometer X-axis data (16-bit) - signed
    CMPS12_REG_ACCRAW_Y        = 0x0E  # Raw accelerometer Y-axis data (16-bit) - signed
    CMPS12_REG_ACCRAW_Z        = 0x10  # Raw accelerometer Z-axis data (16-bit) - signed
    CMPS12_REG_GYRORAW_X       = 0x12  # Raw gyroscope X-axis data (16-bit) - signed
    CMPS12_REG_GYRORAW_Y       = 0x14  # Raw gyroscope Y-axis data (16-bit) - signed
    CMPS12_REG_GYRORAW_Z       = 0x16  # Raw gyroscope Z-axis data (16-bit) - signed
    CMPS12_REG_TEMP            = 0x18  # Temperature data (16-bit) - signed
    CMPS12_REG_CALSTAT         = 0x1E  # Calibration state, bits 0 and 1 reflect the calibration status (0 = un-calibrated, 3 = fully calibrated)
    CMPS12_REG_BEARING_BOSCH   = 0x1A  # Compass Bearing 16 bit This is the angle Bosch generate in the BNO055 (0-5759), divide by 16 for degrees
    CMPS12_REG_PITCH16         = 0x1C  # Pitch angle 16 bit - signed byte giving angle in degrees from the horizontal plane (+/-180°)
    
    def begin(self):
        version = self._read_register(self.CMPS12_REG_COMMAND)
        return version == 5 

    def get_bearing_360(self):
        return self._read_register(self.CMPS12_REG_BEARING16, 2) / 10.0

    def get_pitch_90(self):
        pitch90 = self._read_register(self.CMPS12_REG_PITCH90)
        pitch90 = pitch90 - 255 if pitch90 > 90 else pitch90
        return pitch90
    
    def get_pitch_180(self):
        pitch16 = self._read_register(self.CMPS12_REG_PITCH16, 2)
        # If the highest bit is set, value is negative (2^16 - 1) -> (180° to -180°) 
        pitch16 = pitch16 - 65535 if pitch16 > 180 else pitch16
        return pitch16

    def get_roll_90(self):
        roll = self._read_register(self.CMPS12_REG_ROLL90)
        # If the highest bit is set, value is negative
        roll = roll - 255 if roll > 90 else roll
        return roll
    
    def get_mag_raw(self):
        mag_x = self._read_register(self.CMPS12_REG_MAGNRAW_X, 2)
        mag_y = self._read_register(self.CMPS12_REG_MAGNRAW_Y, 2)
        mag_z = self._read_register(self.CMPS12_REG_MAGNRAW_Z, 2)
        # mag_x = mag_x - 65535 if (mag_x >= 1280 or mag_x <= 144) else mag_x # BNO055 datasheet
        # mag_y = mag_y - 65535 if (mag_y >= 1280 or mag_y <= 144) else mag_y
        # mag_z = mag_z - 65535 if (mag_z >= 1280 or mag_z <= 144) else mag_z
        return (mag_x, mag_y, mag_z)
    
    def get_acc_raw(self):
        acc_x = self._read_register(self.CMPS12_REG_ACCRAW_X, 2)
        acc_y = self._read_register(self.CMPS12_REG_ACCRAW_Y, 2)
        acc_z = self._read_register(self.CMPS12_REG_ACCRAW_Z, 2)
        # acc_x = acc_x - 65535 if acc_x >= 2048 else acc_x
        # acc_y = acc_y - 65535 if acc_y >= 2048 else acc_y
        # acc_z = acc_z - 65535 if acc_z >= 2048 else acc_z
        return (acc_x, acc_y, acc_z)
    
    def get_gyro_raw(self):
        gyro_x = self._read_register(self.CMPS12_REG_GYRORAW_X, 2)
        gyro_y = self._read_register(self.CMPS12_REG_GYRORAW_Y, 2)
        gyro_z = self._read_register(self.CMPS12_REG_GYRORAW_Z, 2)
        # gyro_x = gyro_x - 65535 if gyro_x >= 2000 else gyro_x
        # gyro_y = gyro_y - 65535 if gyro_y >= 2000 else gyro_y
        # gyro_z = gyro_z - 65535 if gyro_z >= 2000 else gyro_z
        return (gyro_x, gyro_y, gyro_z)
    
    def get_bearing_bosch_BNO055(self):
        return self._read_register(self.CMPS12_REG_BEARING_BOSCH, 2) / 16.0
        

    def get_temp(self):
        temp = self._read_register(self.CMPS12_REG_TEMP,2)
        return temp
    
    def get_cal_stat_raw(self):
        return self._read_register(self.CMPS12_REG_CALSTAT)
    
    def get_cal_stat(self):
        cal = self.get_cal_stat_raw()
        if cal == 0xFF:
            return CalStat.CAL_HIGH
        cal_s = cal >> 6                # Extract the 2 most significant bits.
        cal_m = cal & 0x03
        if (cal_s == 0) or (cal_m == 0):
            return CalStat.CAL_LOW
        return CalStat.CAL_MED

    def store_cal(self):
        sequence = [self.address, 0xF0, 0xF5, 0xF6, 0x00]
        self._write_sequence(sequence)
        print("Calibration stored")
        return True

    def erase_cal(self):
        sequence = [self.address, 0xE0, 0xE5, 0xE2, 0x00]
        self._write_sequence(sequence)
        return True

# ------------------------------------------------------------
class CMPS14(CMPS1x):
    CMPS14_REG_COMMAND      = 0x00
    CMPS14_REG_BEARING255   = 0x01
    CMPS14_REG_BEARING16    = 0x02
    # CMPS14_REG_PITCH90      = 0x04
    # CMPS14_REG_ROLL90       = 0x05
    # CMPS14_REG_MAGN_X       = 0x06
    # CMPS14_REG_MAGN_Y       = 0x08
    # CMPS14_REG_MAGN_Z       = 0x0A
    # CMPS14_REG_LINACC_X     = 0x0C
    # CMPS14_REG_LINACC_Y     = 0x0E
    # CMPS14_REG_LINACC_Z     = 0x10
    # CMPS14_REG_GYRORAW_X    = 0x12
    # CMPS14_REG_GYRORAW_Y    = 0x14
    # CMPS14_REG_GYRORAW_Z    = 0x16
    CMPS14_REG_ROLL16       = 0x1C
    CMPS14_REG_CALSTAT      = 0x1E
	# V5+ commands
    CMPS14_REG_PITCH16      = 0x1A
    # CMPS14_REG_ACC16_X      = 0x1F
    # CMPS14_REG_ACC16_Y      = 0x21
    # CMPS14_REG_ACC16_Z      = 0x23
    # CMPS14_REG_GYRO16_X     = 0x25
    # CMPS14_REG_GYRO16_X     = 0x27
    # CMPS14_REG_GYRO16_X     = 0x29
 

    def begin(self):
        version = self._read_register(self.CMPS14_REG_COMMAND)
        return version == 7

    def get_bearing16(self):
        return self._read_register(self.CMPS14_REG_BEARING16, 2) / 10.0

    def get_pitch16(self):
        pitch16 = self._read_register(self.CMPS14_REG_PITCH16, 2)
        # If the highest bit is set, value is negative (2^16 - 1) -> (180° to -180°)
        pitch16 = pitch16 - 65535 if pitch16 >= 180 else pitch16
        return pitch16 / 10.0

    def get_roll16(self):
        roll16 = self._read_register(self.CMPS14_REG_ROLL16, 2)
        # If the highest bit is set, value is negative (2^16 - 1) -> (180° to -180°)
        roll16 = roll16 - 65535 if roll16 >= 180 else roll16
        return roll16 / 10.0

    def store_cal(self):
        sequence = [self.address, 0xF0, 0xF5, 0xF6, 0x00]
        self._write_sequence(sequence)
        return True

    def erase_cal(self):
        sequence = [self.address, 0xE0, 0xE5, 0xE2, 0x00]
        self._write_sequence(sequence)
        return True

    def set_cal_config(self, periodic_save=False, gyrocal=False, accelcal=False, magcal=False):
        settings = 0x80
        if periodic_save:
            settings |= 0x10
        if gyrocal:
            settings |= 0x04
        if accelcal:
            settings |= 0x02
        if magcal:
            settings |= 0x01
        sequence = [self.address, 0x98, 0x95, 0x99, settings]
        self._write_sequence(sequence)
        return True
