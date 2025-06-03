import logging

import nxt.motor as Motor
import nxt.sensor as Sensor

from nxt.brick import Brick

from ln3d_scanner.timer import LN3DTimer
from ln3d_scanner.nxt.motors import PrecisionMotor, InvertedMotor, DualMotors
from ln3d_scanner.nxt.sensors import Switch


logger = logging.getLogger(__name__)


class CameraBar(LN3DTimer):

    def __init__(self, brick: Brick, motor_one_port: Motor.Port, motor_two_port: Motor.Port, touch_port: Sensor.Port, power: int = 100, 
                 direction: int = -1, camera_stop_offset: int = 17320, gear_ratio: int = 120, **kwargs):
        super().__init__(**kwargs)
        self.brick = brick
        self.motors = DualMotors(PrecisionMotor(brick, motor_one_port), InvertedMotor(brick, motor_two_port))
        self.motors.reset_position(True)
        self.camera_stop = Switch(brick, touch_port)
        
        self._power = power
        self._up_direction = direction
        self._camera_stop_offset = camera_stop_offset
        self._gear_ratio = gear_ratio  # From motor to camera bar ratio.
        

    @property
    def power(self) -> int:
        """ Returns the power direction away from home. """
        return self._power * self._up_direction
    
    @power.setter
    def power(self, value: int):
        self._power = abs(value)  # Use positive values only.
    
    def invert_up_direction(self):
        self._up_direction *= -1

    def up(self):
        """ Moves the camera bar up some degree. This function uses dual motor turns and must require a sleep after for accurate results. """
        self.motors.turn(self.power, 90)

    def home(self):
        """ 
            Homes the camera bar using z stop.
            WARNING:
                If the camera bar is incorrectly setup, Click the camera stop to reverse direction.
                Hold for 1 second or until the tone is played to abort homing. Must release to abort, holding longer will assume camera bar is homed.
        """
        self.motors.run(-self.power)  # Use inverted power
        state = None
        while True:
            with self.camera_stop as switch:
                # Use enter method to update once per loop and not on each function call.
                if switch.is_pressed():
                    # Stop motors as soon as switch is pressed.
                    state = 'reverse'
                    self.motors.stop()
                if switch.is_pressed(1000):
                    self.brick.play_tone(440, 500)
                    state = 'abort'
                if switch.is_pressed(2000):
                    # If switch reaches 2 seconds, the camera bar is homed.
                    state = None  # Reset state.
                    break
                if state is not None and switch.is_released():
                    break
            self.wait()  # Allow processing time between each loop.

        if state == 'reverse':
            logger.info('Reverse homing direction.')
            self.invert_up_direction()
            self.home()  # Re home.
        elif state =='abort':
            logger.info('Abort homing procedure')
        else:
            self.brick.play_sound_file(False, '! Backup.rso')
            self.motors.turn(self.power, 360)
            logger.info('Homed camera bar.')
    

    def calibrate_camera_offset(self):
        """ 
            Run this function if you wish to calibrate the camera offset. 
            Procedure:
                1) Home the camera bar to camera stop.
                2) Run the camera bar in the up direction.
                3) Press the camera stop switch when the bar is in the at the top (12 o clock) position.
                4) Adjust the camera bar to be exactly centered using the two wheels to rotate each motor.
                5) Press the camera stop button again to return home and print out the rotational values and update internal values.
                6) Camera will home again.
        """
        logger.info('Calibrating camera bar rotation offset.')
        self.home()
        # only limit to half a camera bar rotation so it won't break anything.
        logger.info('Reverse camera rotating till center with a maximum of 180 degrees camera bar rotation.')
        self.motors.turn(self.power, 180 * self._gear_ratio, stop_turn=self.camera_stop.is_pressed)
        logger.info('Waiting for user input to continue.')
        self.camera_stop.wait_for_press(timeout=120)  # Allow 2 minutes of calibatrion.
        logger.info('User calibrated center. Returning home.')
        self.motors.reset_position(True)
        self.home()
        rotation = self.motors.get_tacho().block_tacho_count
        logger.info(f'Camera calibration results:\n  Rotation from center to camera stop: {rotation}\n  Up direction: {self._up_direction}')
        self._camera_stop_offset = rotation

        
