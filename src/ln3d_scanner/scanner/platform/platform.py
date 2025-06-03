import nxt.motor as Motor
from nxt.brick import Brick

from ln3d_scanner.timer import LN3DTimer


class Platform(LN3DTimer):

    def __init__(self, brick: Brick, motor_port: Motor.Port, frequency = 30):
        super().__init__(frequency)
        self.brick = brick
        self.motor = brick.get_motor(motor_port)
    
    
    
