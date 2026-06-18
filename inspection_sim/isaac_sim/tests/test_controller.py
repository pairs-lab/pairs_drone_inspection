import numpy as np
from drone.controller import Pid, motor_mixing, position_to_thrust

def test_pid_drives_error_to_zero_sign():
    pid = Pid(kp=1.0, ki=0.0, kd=0.0)
    # positive error -> positive output
    assert pid.step(error=2.0, dt=0.01) > 0
    pid.reset()
    assert pid.step(error=-2.0, dt=0.01) < 0

def test_pid_derivative_opposes_change():
    pid = Pid(kp=0.0, ki=0.0, kd=1.0)
    pid.step(error=0.0, dt=0.1)
    out = pid.step(error=1.0, dt=0.1)  # error rising -> derivative positive
    assert out > 0

def test_motor_mixing_pure_thrust_is_equal():
    # zero torque -> all 4 rotors equal, sum == total thrust
    m = motor_mixing(total_thrust=4.0, tx=0.0, ty=0.0, tz=0.0)
    assert len(m) == 4
    assert all(abs(v - 1.0) < 1e-6 for v in m)
    assert abs(sum(m) - 4.0) < 1e-6

def test_motor_mixing_roll_torque_differs_left_right():
    m = motor_mixing(total_thrust=4.0, tx=1.0, ty=0.0, tz=0.0)
    # roll torque should make opposite rotors differ
    assert not all(abs(v - m[0]) < 1e-6 for v in m)
    assert all(v >= 0 for v in m)  # thrusts clamped non-negative

def test_position_to_thrust_hover_points_up():
    # at rest under target directly above -> desired thrust vector has +Z dominant
    f = position_to_thrust(pos=np.array([0,0,0.0]), vel=np.zeros(3),
                           target=np.array([0,0,1.0]), mass=1.0, g=9.81)
    assert f[2] > f[0] and f[2] > f[1]
