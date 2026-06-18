import numpy as np

class Pid:
    def __init__(self, kp, ki, kd, i_limit=10.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit = i_limit
        self.reset()
    def reset(self):
        self._i = 0.0
        self._prev = None
    def step(self, error, dt):
        self._i = float(np.clip(self._i + error * dt, -self.i_limit, self.i_limit))
        d = 0.0 if self._prev is None else (error - self._prev) / dt
        self._prev = error
        return self.kp * error + self.ki * self._i + self.kd * d

def position_to_thrust(pos, vel, target, mass, g, kp=6.0, kd=4.0):
    """PD on position -> desired world thrust vector (N). Adds gravity feedforward."""
    acc_des = kp * (target - pos) - kd * vel
    acc_des = acc_des + np.array([0.0, 0.0, g])  # counter gravity
    return mass * acc_des

# rotor layout (X config), unit arm; columns: [FL, FR, RL, RR]
# thrust = total/4 + roll/pitch/yaw mixing
def motor_mixing(total_thrust, tx, ty, tz):
    base = total_thrust / 4.0
    # signs per rotor for roll(tx), pitch(ty), yaw(tz)
    mix = [
        base + (-tx + ty + tz) / 4.0,  # FL
        base + (+tx + ty - tz) / 4.0,  # FR
        base + (-tx - ty - tz) / 4.0,  # RL
        base + (+tx - ty + tz) / 4.0,  # RR
    ]
    return [max(0.0, v) for v in mix]
