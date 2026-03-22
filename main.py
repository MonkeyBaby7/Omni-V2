from machine import Pin, I2C, UART
import time

from imu_bno055 import find_bno055
from omni_drive import OmniRobot
from nextion_ui import Nextion

STARTUP_DELAY_S = 2.0

CONTROL_LOOP_MS = 30
SCREEN_UPDATE_MS = 120
HEADING_SMOOTH_ALPHA = 0.25

# Sensor ring on UART1 GP4/GP5
BALL_UART_ID = 1
BALL_UART_BAUD = 115200
BALL_UART_TX_PIN = 4
BALL_UART_RX_PIN = 5

BALL_POWER = 0.70
BALL_TIMEOUT_MS = 300


def smooth_angle_deg(prev, new, alpha):
    if prev is None:
        return new
    err = (new - prev + 180.0) % 360.0 - 180.0
    return (prev + alpha * err) % 360.0


def read_ball_heading(uart):
    if not uart.any():
        return None

    data = uart.readline()
    if not data:
        return None

    print("RAW UART:", data)

    try:
        text = data.decode().strip()
    except Exception:
        print("Decode failed")
        return None

    print("TEXT:", text)

    if not text:
        return None

    if text.lower() == "no ball detected":
        return -1

    try:
        return int(text) % 360
    except Exception:
        print("Bad UART data:", text)
        return None


def log_both(nx, text):
    print(text)
    nx.log(text)


if __name__ == "__main__":
    time.sleep(STARTUP_DELAY_S)

    # ---------------------------
    # Display startup
    # ---------------------------
    nx = Nextion(uart_id=0, tx_pin=16, rx_pin=17, baud=9600)
    nx.goto_page(0)
    nx.clear_log()
    nx.set_text("t0", "BOOTING...")
    log_both(nx, "BOOT")

    # ---------------------------
    # Robot motors
    # ---------------------------
    robot = OmniRobot()
    robot.stop_all()
    log_both(nx, "MOTORS OK")

    # ---------------------------
    # IMU startup
    # ---------------------------
    nx.set_text("t0", "STARTING I2C...")
    log_both(nx, "I2C START")

    i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)

    devices = i2c.scan()
    device_list = [hex(x) for x in devices]
    print("I2C devices:", device_list)
    nx.log("I2C " + str(device_list))

    nx.set_text("t0", "FINDING IMU...")
    log_both(nx, "FIND IMU")

    imu = find_bno055(i2c)
    if imu is None:
        nx.set_text("t0", "IMU NOT FOUND")
        log_both(nx, "IMU FAIL")
        robot.stop_all()
        raise SystemExit

    log_both(nx, "IMU OK")
    nx.set_text("t0", "IMU OK")

    time.sleep(1.0)

    hold_heading = imu.heading_deg()
    hold_text = "HOLD {:.1f}".format(hold_heading)
    log_both(nx, hold_text)

    # ---------------------------
    # Ball UART startup
    # ---------------------------
    nx.set_text("t0", "STARTING BALL UART...")
    log_both(nx, "BALL UART START")

    ball_uart = UART(
        BALL_UART_ID,
        BALL_UART_BAUD,
        tx=Pin(BALL_UART_TX_PIN),
        rx=Pin(BALL_UART_RX_PIN)
    )

    log_both(nx, "BALL UART OK")
    nx.set_text("t0", "READY")

    # ---------------------------
    # Runtime state
    # ---------------------------
    next_control = time.ticks_ms()
    next_screen = time.ticks_ms()

    filtered_heading = None
    ball_heading = -1
    last_ball_ms = time.ticks_ms()

    status = "NO BALL"
    last_printed_status = ""
    last_printed_ball = None

    # ---------------------------
    # Main loop
    # ---------------------------
    while True:
        now = time.ticks_ms()

        # Read UART data from sensor ring
        new_heading = read_ball_heading(ball_uart)
        if new_heading is not None:
            ball_heading = new_heading
            last_ball_ms = now

            if ball_heading == -1:
                if last_printed_ball != -1:
                    log_both(nx, "NO BALL DETECTED")
                    last_printed_ball = -1
            else:
                if last_printed_ball != ball_heading:
                    log_both(nx, "BALL {}".format(ball_heading))
                    last_printed_ball = ball_heading

        # Timeout if signal stops arriving
        if time.ticks_diff(now, last_ball_ms) > BALL_TIMEOUT_MS:
            ball_heading = -1

        # Control loop
        if time.ticks_diff(now, next_control) >= 0:
            next_control = now + CONTROL_LOOP_MS

            raw_yaw = imu.heading_deg()
            filtered_heading = smooth_angle_deg(
                filtered_heading,
                raw_yaw,
                HEADING_SMOOTH_ALPHA
            )

            if ball_heading == -1:
                robot.stop_all()
                status = "NO BALL"
            else:
                rot = robot.heading_hold_rot(
                    hold_heading,
                    filtered_heading,
                    moving=True
                )

                robot.drive_heading_power(ball_heading, BALL_POWER, rot)
                status = "BALL {:03d} ROT {:.2f}".format(ball_heading, rot)

        # Print state changes to serial and display log
        if status != last_printed_status:
            log_both(nx, status)
            last_printed_status = status

        # Update display status line
        if filtered_heading is not None and time.ticks_diff(now, next_screen) >= 0:
            next_screen = now + SCREEN_UPDATE_MS
            nx.status(filtered_heading, status)

        time.sleep_ms(1)
