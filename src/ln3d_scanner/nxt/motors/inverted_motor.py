from nxt.motor import get_tacho_and_state

from .precision_motor import PrecisionMotor


class InvertedMotor(PrecisionMotor):

    def _set_state(self, state):
        state.power *= -1  # Invert state power.
        #state.tacho_limit *= -1  # Inver tacho limit.
        return super()._set_state(state)
    
    def _read_state(self):
        """ Invert power back to default values. """
        values = self.brick.get_output_state(self.port)
        state, tacho = get_tacho_and_state(values)
        state.power *= -1  # Invert
        # state.tacho_limit *= -1
        self._state = state
        tacho.tacho_count *= -1
        if tacho.rotation_count is not None:
            tacho.rotation_count *= -1
        if tacho.block_tacho_count is not None:
            tacho.block_tacho_count *= -1
        return self._state, tacho
    