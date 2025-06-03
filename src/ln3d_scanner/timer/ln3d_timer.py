import time

from typing import Optional


class LN3DTimer:
    """
        Mixing type class.
    """

    def __init__(self, frequency: int = 30):
        self.frequency = frequency

    def wait(self, duration: Optional[int] = None):
        """ 
            Waits the object till the timer runs out. 
            If duration is provided it will wait the given seconds instead.
            Otherwise 1 over frquency is used.
        """
        if duration is None:
            duration = 1 / self.frequency
        time.sleep(duration)

    def now(self) -> int:
        """ Return time in nano seconds. """
        return time.time()