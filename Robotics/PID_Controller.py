import math
import matplotlib.pyplot as plt

class PidController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.e_i = 0.0
        self.prev_e = 0.0

    def time_step(self, r, y, dt):
        
