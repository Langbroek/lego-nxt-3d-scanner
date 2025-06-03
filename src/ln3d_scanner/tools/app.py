import libusb_package
import time
import logging


from typing import List, Optional, Tuple

import nxt.locator
import nxt.motor as Motor
import nxt.sensor as Sensor
from nxt.sensor.generic import Touch

from ln3d_scanner.nxt.motors import PrecisionMotor, InvertedMotor, DualMotors
from ln3d_scanner.nxt.sensors import Switch
from ln3d_scanner.scanner.camera import CameraBar


logging.basicConfig()
logging.getLogger().setLevel(logging.INFO)


if __name__ == '__main__':

    libusb_package.get_libusb1_backend()

    with nxt.locator.find() as brick:

        # Once found, print its name.
        print("Found brick:", brick.get_device_info()[0])
        # And play a recognizable note.
        print('volt', brick.get_battery_level())
        
        platform = InvertedMotor(brick, Motor.Port.A)
        platform.reset_position(False)

        camera_bar = CameraBar(brick, Motor.Port.B, Motor.Port.C, Sensor.Port.S1)
        try:
            # camera_bar.home()
            camera_bar.calibrate_camera_offset()
        finally:
            camera_bar.motors.stop()
            time.sleep(1)

        time.sleep(1)
