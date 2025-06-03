from typing_extensions import Self

from nxt.motor import TachoInfo, OutputState


from ln3d_scanner.timer import LN3DTimer
from .precision_motor import PrecisionMotor


class StateMismatchException(Exception):
    pass


class DualState:

    def __init__(self, leader_state: OutputState, follower_state: OutputState):
        self.leader = leader_state
        self.follower = follower_state

    def _get_value(self, property: str):
        """ Get value. Raises exception on mismatch. """
        leader_value = getattr(self.leader, property)
        follower_value = getattr(self.follower, property)
        if leader_value != follower_value:
            raise StateMismatchException()
        return leader_value
    
    def _set_value(self, property: str, value):
        """ Sets value for both leader and follower. """
        setattr(self.leader, property, value)
        setattr(self.follower, property, value)
    
    @property
    def power(self):
        return self._get_value('power')
    
    @property
    def mode(self):
        return self._get_value('mode')

    @property
    def regulation_mode(self):
        return self._get_value('regulation_mode')
    
    @property
    def turn_ratio(self):
        return self._get_value('turn_ratio')
    
    @property
    def run_state(self):
        return self._get_value('run_state')
    
    @property
    def tacho_limit(self):
        return self._get_value('tacho_limit')
    
    @power.setter
    def power(self, value):
        self._set_value('power', value)
    
    @mode.setter
    def mode(self, value):
        self._set_value('mode', value)

    @regulation_mode.setter
    def regulation_mode(self, value):
        self._set_value('regulation_mode', value)

    @turn_ratio.setter
    def turn_ratio(self, value):
        self._set_value('turn_ratio', value)

    @run_state.setter
    def run_state(self, value):
        return self._set_value('run_state', value)
    
    @tacho_limit.setter
    def tacho_limit(self, value):
        return self._set_value('tacho_limit', value)
    


class DualTacho(TachoInfo):

    def __init__(self, leader_tacho: TachoInfo, follower_tacho: TachoInfo, leader_motor: PrecisionMotor, follower_motor: PrecisionMotor):
        self.leader = leader_tacho
        self.leader_motor = leader_motor
        self.follower = follower_tacho
        self.follower_motor = follower_motor
    
    @property
    def tacho_count(self) -> int:
        """ Returns the average tacho count. """
        return int((self.leader.tacho_count + self.follower.tacho_count) / 2)
    
    @property
    def block_tacho_count(self) -> int:
        """ Return the average block tacho count. """
        leader_count = 0 if self.leader.block_tacho_count is None else self.leader.block_tacho_count
        follower_count = 0 if self.follower.block_tacho_count is None else self.follower.block_tacho_count
        return int((leader_count + follower_count) / 2)
    
    @property
    def rotation_count(self) -> int:
        """ Return the average rotation count. """
        leader_count = 0 if self.leader.rotation_count is None else self.leader.rotation_count
        follower_count = 0 if self.follower.rotation_count is None else self.follower.rotation_count
        return int((leader_count + follower_count) / 2)

    def get_target(self, tacho_limit, direction) -> Self:
        """Returns a TachoInfo object which corresponds to tacho state after
        moving for tacho_limit ticks in the given direction.

        :param int tacho_limit: Move angle.
        :param int direction: 1 (add) or -1 (subtract).
        :return: Updated state.
        :rtype: TachoInfo
        """
        # TODO: adjust other fields
        if abs(direction) != 1:
            raise ValueError("invalid direction")
        
        return DualTacho(
            TachoInfo([self.leader.tacho_count + direction * tacho_limit, None, None]),
            TachoInfo([self.follower.tacho_count + direction * tacho_limit, None, None]),
            self.leader_motor,
            self.follower_motor
        )

    def is_greater(self, target: Self, direction: int, motor: PrecisionMotor = None) -> bool:
        """ If motor is provided, return if the specific motor is near otherwise return if both are averagely near. """
        if motor == self.leader_motor:
            return direction * (self.leader.tacho_count - target.leader.tacho_count) > 0
        elif motor == self.follower_motor:
            return direction * (self.follower.tacho_count - target.follower.tacho_count) > 0
        # Average
        return direction * (self.tacho_count - target.tacho_count) > 0

    def is_near(self, target: Self, threshold: int, motor: PrecisionMotor = None):
        """ If motor is provided return the specific motor is near the target. """
        if motor == self.leader_motor:
            return abs(target.leader.tacho_count - self.leader.tacho_count) < threshold
        elif motor == self.follower_motor:
            return abs(target.follower.tacho_count - self.follower.tacho_count) < threshold
        return abs(target.tacho_count - self.tacho_count) < threshold

    def __str__(self):
        return f"Dual tacho stats: \n  Leader: {str(self.leader)}\n  Follower: {str(self.follower)}"


class DualMotors(PrecisionMotor):

    def __init__(self, leader: PrecisionMotor, follower: PrecisionMotor, **kwargs):
        LN3DTimer.__init__(self, **kwargs)  # Skip precision motor init.
        self.leader = leader
        self.follower = follower

    @property
    def method(self) -> str:
        return self.leader.method

    def reset_position(self, relative):
        self.leader.reset_position(relative)
        self.follower.reset_position(relative)

    def _get_new_state(self) -> DualState:
        """ Careful the follower may not like the leaders state. """
        return DualState(
            self.leader._get_new_state(),
            self.follower._get_new_state()
        )

    def _set_state(self, state: DualState):
        self.leader._set_state(state.leader)
        self.follower._set_state(state.follower)

    def brake(self):
        self.leader.brake()
        self.follower.brake()
    
    def run(self, power=100):
        """Warning! After calling this method, make sure to call idle. The
        motors are reported to behave wildly otherwise.
        """
        self.leader.run(power, True)
        self.follower.run(power, True)

    def idle(self):
        """ Idle both motors. """
        self.leader.idle()
        self.follower.idle()
    
    def _eta(self, tacho, target, power):
        """ Returns the average eta. Not accurate. """
        m1e = self.leader._eta(tacho, target, power)
        m2e = self.follower._eta(tacho, target, power)
        eta = (m1e + m2e) / 2
        return eta

    def _is_blocked(self, tacho: DualTacho, last_tacho: DualTacho, direction: int, motor: PrecisionMotor = None):
        """ Return if the dual motors are blocked or specific motor if provided. """
        if motor == self.leader:
            return self.leader._is_blocked(tacho.leader, last_tacho.leader, direction)
        elif motor == self.follower:
            return self.follower._is_blocked(tacho.follower, last_tacho.follower, direction)
        return self.leader._is_blocked(tacho.leader, last_tacho.leader, direction) or self.follower._is_blocked(tacho.follower, last_tacho.follower, direction)

    
    def get_tacho(self) -> DualTacho:
        """ Returns an average tacho of both motors. """
        leader_tacho = self.leader.get_tacho()
        follower_tacho = self.follower.get_tacho()
        return DualTacho(leader_tacho, follower_tacho, self.leader, self.follower)

    def turn(self, power, tacho_units, brake: bool = True, stop_turn = lambda: False, timeout: int = 1):
        """ 
            Override turn method. We cant to run motors separately and not averaged. 
            If one motor spins more than the other the next time it can spin slightly less to keep up.
            Disable emulation on dual motors.
        """
        tacho_limit = tacho_units
    
        if tacho_limit < 0:
            raise ValueError("tacho_units must be greater than 0!")
        
        # Use a bound of 10 as minimum otherwise moters won't run.
        tacho_limit = max(10, tacho_limit)
        
        threshold = self.get_threshold()

        tacho = self.get_tacho()
        state = self._get_new_state()  # Get a dual state.

        # Update modifiers even if they aren't used, might have been changed
        state.power = power

        self._set_state(state)

        direction = 1 if power > 0 else -1

        tacho_target = tacho.get_target(tacho_limit, direction)
        blocked = False

        current_time = self.now()
        last_time = current_time

        # Turn states.
        motor_states = {self.leader: True, self.follower: True}
        is_motor_running = lambda motor: motor_states[motor] == True  # Explicit.

        def stop_motor(motor: PrecisionMotor):
            motor_states[motor] = False
            if brake:
                motor.brake()
            else:
                motor.idle()
            
        while not stop_turn() and (is_motor_running(self.leader) or is_motor_running(self.follower)):
            # Returns if leader or follower is near, which ever comes first.
            if not tacho.is_near(tacho_target, 100):
                # Sleep longer when not near.
                self.wait(.1)
            else:
                self.wait()

            if not blocked:  # if still blocked, don't reset the counter
                last_tacho = tacho
                last_time = current_time
                current_time = self.now()

            tacho = self.get_tacho()
            blocked = self._is_blocked(tacho, last_tacho, direction)  # TODO replace

            if (current_time - last_time) > timeout:
                # The motor can be up to 80+ degrees in either direction from target
                # when using Bluetooth.
                if is_motor_running(self.leader) and self._is_blocked(tacho, last_tacho, direction, motor=self.leader):
                    stop_motor(self.leader)
                if is_motor_running(self.follower) and self._is_blocked(tacho, last_tacho, direction, motor=self.follower):
                    stop_motor(self.follower)
            else:
                # Check if motors are near target to stop them.
                if tacho.is_near(tacho_target, threshold, motor=self.leader) or tacho.is_greater(tacho_target, direction, motor=self.leader):
                    stop_motor(self.leader)
                if tacho.is_near(tacho_target, threshold, motor=self.follower) or tacho.is_greater(tacho_target, direction, motor=self.follower):
                    stop_motor(self.follower)
        
        if brake:
            self.stop()

    def stop(self):
        """ Stops motors. """
        self.brake()
        self.wait(.5)  # wait for .5 seconds to brake correctly before idle.
        self.idle()

