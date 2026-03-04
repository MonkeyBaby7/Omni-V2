# main.py
# ------------------------------------------------------------
# MAIN PROGRAM for omni robot
#
# You only need to edit the COMMANDS list.
# Each command is 3 values:
#   (heading_code, power, duration_seconds)
#
# heading_code:
#   0..359  = translate in that direction
#   360     = rotate CCW in place (power = rotation power)
#   361     = rotate CW in place  (power = rotation power)
#   362     = hold heading (robot tries to face startup direction)
# ------------------------------------------------------------

from machine import Pin, I2C
import time

from imu_bno055 import find_bno055
from omni_drive import OmniRobot
from nextion_ui import Nextion

STARTUP_DELAY_S = 2.0

# How often we update motors + heading (ms timing is reliable)
CONTROL_LOOP_MS = 30   # ~33 Hz
SCREEN_UPDATE_MS = 100 # 10 Hz

# Optional: smooth heading to reduce jitter.
# 0.0 = no smoothing, 0.2..0.4 = typical
HEADING_SMOOTH_ALPHA = 0.25

# ------------------------------------------------------------
# EDIT ONLY THIS LIST (three values per command)
# ------------------------------------------------------------
COMMANDS = [
    (0,   0.70, 1.0),
    (180,  0.70, 1.0),
    (90,  0.70, 1.0),
    (270,  0.70, 2.0),
    (90,  0.70, 1.0),
    (45,  0.70, 1.0),
    (225,  0.70, 1.0),
    (315,  0.70, 1.0),
    (135,  0.70, 1.0),
    (360, 0.7, 2.0),
    (361, 0.7, 2.0),
    (362, 0.00, 20.0),
]
# ------------------------------------------------------------


def fmt_cmd(h, p, t):
    return f"H{h} P{p:.2f} T{t:.1f}"


def clamp01(x):
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


def smooth_angle_deg(prev, new, alpha):
    """
    Smooth heading without breaking wrap-around at 0/360.
    We smooth the smallest error direction.
    """
    if prev is None:
        return new
    err = (new - prev + 180.0) % 360.0 - 180.0
    return (prev + alpha * err) % 360.0


def run_one_command(robot, imu, nx, hold_heading, heading_code, power, duration_s):
    """
    Runs one command for duration_s seconds.
    Shows live heading/status on t0 and logs START/DONE on t1.
    """
    power = clamp01(power)

    nx.log("START " + fmt_cmd(heading_code, power, duration_s))

    start_ms = time.ticks_ms()
    end_ms = start_ms + int(duration_s * 1000)

    next_control = start_ms
    next_screen = start_ms

    filtered_heading = None

    while time.ticks_diff(end_ms, time.ticks_ms()) > 0:
        now = time.ticks_ms()

        # ---- control loop timing ----
        if time.ticks_diff(now, next_control) >= 0:
            next_control = now + CONTROL_LOOP_MS

            raw_yaw = imu.heading_deg()
            filtered_heading = smooth_angle_deg(filtered_heading, raw_yaw, HEADING_SMOOTH_ALPHA)

            # ---- drive logic ----
            if 0 <= heading_code <= 359:
                rot = robot.heading_hold_rot(hold_heading, filtered_heading, moving=True)
                robot.drive_heading_power(heading_code, power, rot)

                status = f"RUN {fmt_cmd(heading_code, power, duration_s)}"

            elif heading_code == 360:
                robot.drive_xy_rot(0.0, 0.0, abs(power))
                status = f"ROT CCW P{power:.2f}"

            elif heading_code == 361:
                robot.drive_xy_rot(0.0, 0.0, -abs(power))
                status = f"ROT CW  P{power:.2f}"

            elif heading_code == 362:
                rot = robot.heading_hold_rot(hold_heading, filtered_heading, moving=False)
                robot.drive_xy_rot(0.0, 0.0, rot)
                status = "HOLD HEADING"

            else:
                robot.stop_all()
                status = "STOP"

        # ---- screen timing ----
        if filtered_heading is not None and time.ticks_diff(now, next_screen) >= 0:
            next_screen = now + SCREEN_UPDATE_MS
            nx.status(filtered_heading, status)

        time.sleep_ms(1)

    robot.stop_all()
    nx.log("DONE  " + fmt_cmd(heading_code, power, duration_s))


if __name__ == "__main__":
    time.sleep(STARTUP_DELAY_S)

    # Nextion screen (t0 + t1)
    nx = Nextion(uart_id=1, tx_pin=8, rx_pin=9, baud=9600)
    nx.goto_page(0)
    nx.clear_log()
    nx.set_text("t0", "BOOTING...")
    nx.log("BOOT")

    # Robot motors
    robot = OmniRobot()

    # IMU on I2C0: SDA=GP0, SCL=GP1
    i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)

    nx.set_text("t0", "FINDING IMU...")
    imu = find_bno055(i2c)
    if imu is None:
        nx.set_text("t0", "IMU NOT FOUND")
        nx.log("IMU NOT FOUND")
        robot.stop_all()
        raise SystemExit

    nx.log("IMU OK")

    # Give the IMU fusion a moment to settle
    time.sleep(1.0)

    # "Hold heading" is set to whatever direction the robot faces at startup
    hold_heading = imu.heading_deg()
    nx.log(f"HOLD {hold_heading:.1f}")
    nx.log("READY")

    # Run the script
    for h, p, t in COMMANDS:
        run_one_command(robot, imu, nx, hold_heading, h, p, t)
        time.sleep(0.2)

    robot.stop_all()
    nx.status(imu.heading_deg(), "STOPPED")
    nx.log("ALL DONE")
