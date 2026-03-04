# omni_drive.py
# ------------------------------------------------------------
# Omni X robot drive module for 4 wheels + DRV8871 drivers.
#
# Wheel layout:
#   M1 = Front Right
#   M2 = Back Right
#   M3 = Back Left
#   M4 = Front Left
#
# Mixer (your confirmed mapping):
#   m1 = (y - x) + rot
#   m2 = (y + x) + rot
#   m3 = (-y + x) + rot
#   m4 = (-y - x) + rot
#
# Axes:
#   x: right +, left -
#   y: forward +, back -
#   rot: CCW +, CW -
# ------------------------------------------------------------

from machine import Pin, PWM
import math

PWM_FREQ = 20000

# If your motors do not move at small values, set this.
# Example: 0.40 means "anything between 0 and 0.40 gets bumped up to 0.40".
MIN_MOTOR_EFFORT = 0.40

# Heading hold tuning
DEADBAND_DEG = 6.0

# While MOVING: gentle correction (prevents strong fighting while translating)
KP_MOVE = 0.01
MAX_ROT_MOVE = 0.12

# While STOPPED: stronger correction, needs minimum effort to overcome stiction
KP_IDLE = 0.02
MIN_ROT_IDLE = 0.40
MAX_ROT_IDLE = 0.70

# Your confirmed translation direction signs
SIGN_M1 =  1
SIGN_M2 = -1
SIGN_M3 = -1
SIGN_M4 =  1

# Gains (M2 is weaker, so boost it ~10%)
GAIN_M1 = 1.00
GAIN_M2 = 1.10
GAIN_M3 = 1.00
GAIN_M4 = 1.00


def _clamp(x, lo, hi):
    return max(min(x, hi), lo)


def _apply_min_effort(v, min_effort):
    """
    If v is non-zero but too small to move the motor, bump it up.
    Keeps direction the same.
    """
    if v == 0.0:
        return 0.0
    if abs(v) < min_effort:
        return min_effort if v > 0 else -min_effort
    return v


class DRV8871Motor:
    """
    DRV8871 typical wiring:
      IN1 -> PWM
      IN2 -> PWM

    We drive it like:
      forward: IN1 PWM, IN2 0
      reverse: IN1 0, IN2 PWM
    """
    def __init__(self, in1_pin, in2_pin, sign=1, gain=1.0):
        self.in1 = PWM(Pin(in1_pin))
        self.in2 = PWM(Pin(in2_pin))
        self.in1.freq(PWM_FREQ)
        self.in2.freq(PWM_FREQ)
        self.sign = 1 if sign >= 0 else -1
        self.gain = gain
        self.stop()

    def run(self, speed):
        # Apply direction sign and gain balance
        speed = speed * self.sign * self.gain

        # Clamp to valid range
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
        # Your pin pairs (M3 moved to GP16/GP17 to free GP8/GP9 for Nextion)
        self.M1 = DRV8871Motor(11, 10, sign=SIGN_M1, gain=GAIN_M1)  # Front Right
        self.M2 = DRV8871Motor(13, 12, sign=SIGN_M2, gain=GAIN_M2)  # Back Right
        self.M3 = DRV8871Motor(16, 17, sign=SIGN_M3, gain=GAIN_M3)  # Back Left
        self.M4 = DRV8871Motor(3,  4,  sign=SIGN_M4, gain=GAIN_M4)  # Front Left

    def stop_all(self):
        self.M1.stop(); self.M2.stop(); self.M3.stop(); self.M4.stop()

    def drive_xy_rot(self, x, y, rot):
        """
        Core mixer.
        After mixing we normalise so none exceed ±1.
        Then apply min-effort bump so motors actually move.
        """
        m1 = (y - x) + rot
        m2 = (y + x) + rot
        m3 = (-y + x) + rot
        m4 = (-y - x) + rot

        # Normalise so max magnitude becomes 1.0
        mx = max(abs(m1), abs(m2), abs(m3), abs(m4), 1e-9)
        if mx > 1.0:
            m1 /= mx; m2 /= mx; m3 /= mx; m4 /= mx

        # Apply minimum motor effort (optional but matches your hardware)
        m1 = _apply_min_effort(m1, MIN_MOTOR_EFFORT)
        m2 = _apply_min_effort(m2, MIN_MOTOR_EFFORT)
        m3 = _apply_min_effort(m3, MIN_MOTOR_EFFORT)
        m4 = _apply_min_effort(m4, MIN_MOTOR_EFFORT)

        self.M1.run(m1); self.M2.run(m2); self.M3.run(m3); self.M4.run(m4)

    def drive_heading_power(self, heading_deg, power, rot=0.0):
        """
        Drive in a compass heading with power 0..1.

        IMPORTANT:
        We scale x/y by (|sin|+|cos|) so diagonals are not faster.
        """
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
        """Smallest signed angle error: result in range [-180, +180]."""
        return (target - current + 180.0) % 360.0 - 180.0

    def heading_hold_rot(self, setpoint_deg, current_deg, moving):
        """
        Returns a rotation command (-1..+1) to correct heading.

        moving=True  => gentle correction so it doesn't fight translation
        moving=False => stronger correction and uses minimum rotation effort
        """
        e = self.angle_error_deg(setpoint_deg, current_deg)

        # Deadband prevents "hunting" (wobbling left/right)
        if abs(e) <= DEADBAND_DEG:
            return 0.0

        if moving:
            rot = KP_MOVE * e
            return _clamp(rot, -MAX_ROT_MOVE, MAX_ROT_MOVE)

        rot = KP_IDLE * e
        rot = _clamp(rot, -MAX_ROT_IDLE, MAX_ROT_IDLE)

        # Minimum rotation to actually move when stopped
        if rot > 0:
            rot = max(rot, MIN_ROT_IDLE)
        else:
            rot = min(rot, -MIN_ROT_IDLE)

        return rot
