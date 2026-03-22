from machine import Pin, PWM
import math

PWM_FREQ = 20000

MIN_MOTOR_EFFORT = 0.40

DEADBAND_DEG = 6.0

KP_MOVE = 0.01
MAX_ROT_MOVE = 0.12

KP_IDLE = 0.02
MIN_ROT_IDLE = 0.40
MAX_ROT_IDLE = 0.70

SIGN_M1 = 1
SIGN_M2 = -1
SIGN_M3 = -1
SIGN_M4 = 1

GAIN_M1 = 1.00
GAIN_M2 = 1.10
GAIN_M3 = 1.00
GAIN_M4 = 1.00


def _clamp(x, lo, hi):
    return max(min(x, hi), lo)


def _apply_min_effort(v, min_effort):
    if v == 0.0:
        return 0.0
    if abs(v) < min_effort:
        return min_effort if v > 0 else -min_effort
    return v


class DRV8871Motor:
    def __init__(self, in1_pin, in2_pin, sign=1, gain=1.0):
        self.in1 = PWM(Pin(in1_pin))
        self.in2 = PWM(Pin(in2_pin))
        self.in1.freq(PWM_FREQ)
        self.in2.freq(PWM_FREQ)

        self.sign = 1 if sign >= 0 else -1
        self.gain = gain

        self.stop()

    def run(self, speed):
        speed = speed * self.sign * self.gain
        speed = _clamp(speed, -1.0, 1.0)

        duty = int(abs(speed) * 65535)

        if speed > 0:
            self.in1.duty_u16(duty)
            self.in2.duty_u16(0)
        elif speed < 0:
            self.in1.duty_u16(0)
            self.in2.duty_u16(duty)
        else:
            self.in1.duty_u16(0)
            self.in2.duty_u16(0)

    def stop(self):
        self.in1.duty_u16(0)
        self.in2.duty_u16(0)


class OmniRobot:
    def __init__(self):
        # Layout:
        #
        #        FRONT
        #
        #   M4             M1
        #
        #
        #   M3             M2
        #
        self.M1 = DRV8871Motor(11, 10, sign=SIGN_M1, gain=GAIN_M1)  # Front Right
        self.M2 = DRV8871Motor(13, 12, sign=SIGN_M2, gain=GAIN_M2)  # Back Right
        self.M3 = DRV8871Motor(15, 14, sign=SIGN_M3, gain=GAIN_M3)  # Back Left
        self.M4 = DRV8871Motor(18, 19, sign=SIGN_M4, gain=GAIN_M4)  # Front Left

    def stop_all(self):
        self.M1.stop()
        self.M2.stop()
        self.M3.stop()
        self.M4.stop()

    def drive_xy_rot(self, x, y, rot):
        m1 = (y - x) + rot
        m2 = (y + x) + rot
        m3 = (-y + x) + rot
        m4 = (-y - x) + rot

        mx = max(abs(m1), abs(m2), abs(m3), abs(m4), 1e-9)
        if mx > 1.0:
            m1 /= mx
            m2 /= mx
            m3 /= mx
            m4 /= mx

        m1 = _apply_min_effort(m1, MIN_MOTOR_EFFORT)
        m2 = _apply_min_effort(m2, MIN_MOTOR_EFFORT)
        m3 = _apply_min_effort(m3, MIN_MOTOR_EFFORT)
        m4 = _apply_min_effort(m4, MIN_MOTOR_EFFORT)

        self.M1.run(m1)
        self.M2.run(m2)
        self.M3.run(m3)
        self.M4.run(m4)

    def drive_heading_power(self, heading_deg, power, rot=0.0):
        power = _clamp(power, 0.0, 1.0)
        rad = math.radians(heading_deg % 360)

        sx = math.sin(rad)
        cy = math.cos(rad)

        denom = abs(sx) + abs(cy)
        if denom < 1e-9:
            x = 0.0
            y = 0.0
        else:
            x = (sx / denom) * power
            y = (cy / denom) * power

        self.drive_xy_rot(x, y, rot)

    @staticmethod
    def angle_error_deg(target, current):
        return (target - current + 180.0) % 360.0 - 180.0

    def heading_hold_rot(self, setpoint_deg, current_deg, moving):
        e = self.angle_error_deg(setpoint_deg, current_deg)

        if abs(e) <= DEADBAND_DEG:
            return 0.0

        if moving:
            rot = KP_MOVE * e
            return _clamp(rot, -MAX_ROT_MOVE, MAX_ROT_MOVE)

        rot = KP_IDLE * e
        rot = _clamp(rot, -MAX_ROT_IDLE, MAX_ROT_IDLE)

        if rot > 0:
            rot = max(rot, MIN_ROT_IDLE)
        else:
            rot = min(rot, -MIN_ROT_IDLE)

        return rot
