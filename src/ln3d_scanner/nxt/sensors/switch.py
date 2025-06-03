import nxt.sensor as Sensor

from nxt.brick import Brick
from nxt.sensor.generic import Touch
from typing_extensions import Self

from ln3d_scanner.timer import LN3DTimer


class Switch(LN3DTimer):
    """
        Wrapper class to act on touch using a timer and duration presses for the touch sensor.
    """

    def __init__(self, brick: Brick, port: Sensor.Port, **kwargs):
        super().__init__(**kwargs)
        self.touch = brick.get_sensor(port, Touch)
        self.start_counter = -1
        self._disabled_dict = {}

        self._pressed_state = None  # Pressed state. can be None true or false

    def _is_pressed(self) -> bool:
        """ Wrapped method to return if the touch sensor is pressed. """
        if self._pressed_state is None:
            return self.touch.is_pressed()  # Query touch but don't store result.
        return self._pressed_state  # Return stored value.
        
    def is_pressed(self, duration: int = 0, reset: bool = False, disable_till_depressed: bool = True):
        """ 
            Return if the switch is pressed.
            Updates internal counter for duration presses, this is based on the last time this function is called.
            duration: time in milliseconds to return true for when pressed. (holding)
            reset: bool whether to reset the counter or continue counting from start press.
            disable_till_depressed: bool wether to disable all other pressed signals until it has been depressed.
        """
        if not self._is_pressed():
            self.start_counter = -1
            self._disabled_dict.clear()
            return False
        if duration <= 0:
            if reset:
                self.start_counter = -1
            disabled = self._disabled_dict.get(duration, False)
            if disable_till_depressed:
                self._disabled_dict[duration] = True
            return not disabled
        counter = self.now()
        if self.start_counter < 0:
            self.start_counter = counter
        if ((counter - self.start_counter) * 1000) > duration:
            if reset:
                self.start_counter = -1
            disabled = self._disabled_dict.get(duration, False)
            if disable_till_depressed:
                self._disabled_dict[duration]  = True
            return not disabled
        return False
    
    def is_released(self) -> bool:
        """ Return if the switch is released. """
        return not self._is_pressed()
    
    def wait_for_press(self, timeout: int = 20):
        """ Hold the program till touch is pressed. Provide timeout in seconds that will break that wait. """
        start = self.now()
        while True:
            # Use real pressed getter, otherwise in context use it will halt for ever.
            if self.touch.is_pressed() or (self.now() - start) > timeout:
                break
            self.wait()
    
    def __enter__(self) -> Self:
        """ When entering, internal pressed state is set. This prevents multiple calls to the sensor in a code block. """
        if self._pressed_state is not None:
            raise ReferenceError('Context is already active')
        self._pressed_state = self.touch.is_pressed()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """ Reset the read state. """
        if self._pressed_state is None:
            raise ReferenceError('Context closed already.')
        self._pressed_state = None