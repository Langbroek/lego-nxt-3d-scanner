from nxt.motor import Motor, BlockedException

from ln3d_scanner.timer import LN3DTimer


class PrecisionMotor(Motor, LN3DTimer):

    def __init__(self, brick, port, **kwargs):
        Motor.__init__(self, brick, port)
        LN3DTimer.__init__(self, **kwargs)

    def get_tacho(self):
        tacho = super().get_tacho()
        return tacho

    def get_threshold(self) -> int:
        # TODO Calibrate the new values for ip socket latency.
        if self.method == "bluetooth":
            threshold = 70
        elif self.method == "usb":
            threshold = 5
        elif self.method == "ipbluetooth":
            threshold = 80
        elif self.method == "ipusb":
            threshold = 15
        else:
            threshold = 30  # compromise
        return threshold

    def turn(self, power: int, tacho_units: int, brake: bool = True, stop_turn = lambda: False, timeout: int = 1, emulate: bool = True):
        """ 
            Rotate motors with more precision. 
            Set the frequency to the number of state requests to make per second. Default is 30 times per second.
        """
        tacho_limit = tacho_units
    
        if tacho_limit < 0:
            raise ValueError("tacho_units must be greater than 0!")
        
        # Use a bound of 10 as minimum otherwise moters won't run.
        tacho_limit = max(10, tacho_limit)
        

        threshold = self.get_threshold()

        tacho = self.get_tacho()
        state = self._get_new_state()

        # Update modifiers even if they aren't used, might have been changed
        state.power = power
        if not emulate:
            state.tacho_limit = tacho_limit

        self._set_state(state)

        direction = 1 if power > 0 else -1

        tacho_target = tacho.get_target(tacho_limit, direction)
        blocked = False

        current_time = self.now()
        last_time = current_time
        try:
            while not stop_turn():
                if not tacho.is_near(tacho_target, 100):
                    # Only sleep when not near.
                    self.wait(0.1)
                else:
                    self.wait()

                if not blocked:  # if still blocked, don't reset the counter
                    last_tacho = tacho
                    last_time = current_time
                    current_time = self.now()

                tacho = self.get_tacho()
                blocked = self._is_blocked(tacho, last_tacho, direction)
                if blocked:
                    # The motor can be up to 80+ degrees in either direction from target
                    # when using Bluetooth.
                    if current_time - last_time > timeout:
                        if tacho.is_near(tacho_target, threshold):
                            break
                        else:
                            raise BlockedException("Blocked!")

                if tacho.is_near(tacho_target, threshold) or tacho.is_greater(
                    tacho_target, direction
                ):
                    break
        finally:
            if brake:
                self.brake()
            else:
                self.idle() 


