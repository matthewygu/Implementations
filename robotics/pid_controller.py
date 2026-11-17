import math
import matplotlib.pyplot as plt

class PIDController:
    """
    Note: PID controllers don't seem to work well for systems that are not modeled by first-order or second-order differential equations.
    """

    def __init__(self, Kp, Ki, Kd):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.e_i = 0.0
        self.prev_e = 0.0

    def time_step(self, r, y, dt):
        e = r - y
        self.e_i += e * dt
        de_dt = (e - self.prev_e) / dt

        self.prev_e = e

        return self.Kp * e + self.Ki * self.e_i + self.Kd * de_dt

def ref(t):
    """Ideal value we are trying to get the system to reach at moment t"""
    return 10 * math.log(t+0.001)

if __name__ == "__main__":
    # Create a system, which we can imagine to be a mass attached to a prismatic joint which we want to reach a certain displacement, the reference displacement ref, and its true displacement is x.
    # Assume the joint is pointing upward, so increasing x is increasing vertical height.
    m = 10 # in kg
    x, v, a = 0.0, 0.0, 0.0

    # Assume the prismatic joint points straight upward and experiences gravity, but no friction
    g = 9.81

    total_time = 100.0 # seconds
    dt = 0.001 # time resolution
    se = 0.0 # accumulates error
    pid = PIDController(200, 100, 150)

    t_list = [i * dt for i in range(int(total_time / dt) + 1)]
    r_list: list = []
    x_list: list = []
    max_os = 0.0

    for t in t_list:
        r = ref(t)
        r_list.append(r)
        x_list.append(x)
        se += (r - x)**2 * dt

        u = pid.time_step(r, x, dt)
        force_control = u
        force_gravity = -g * m
        force_net = force_control + force_gravity

        a = force_net / m
        v += a * dt
        x += v * dt

        # Overshoot
        if abs(r-x) > abs(max_os):
            max_os = r-x

    print(f"Squared Error: {se}")
    print(f"Overshoot: {max_os}")
    plt.plot(t_list, r_list, label="reference")
    plt.plot(t_list, x_list, label="true x")
    plt.legend()
    plt.show()
