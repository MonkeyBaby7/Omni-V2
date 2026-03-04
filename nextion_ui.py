# nextion_ui.py
# ------------------------------------------------------------
# Nextion helper for:
#   t0 = status line (heading + current action)
#   t1 = scrolling log
#
# Uses raw bytes so CRLF multi-line works reliably.
# ------------------------------------------------------------

from machine import UART, Pin
import time

_NEXTION_END = b"\xff\xff\xff"


class Nextion:
    def __init__(self, uart_id=1, tx_pin=8, rx_pin=9, baud=9600):
        self.uart = UART(uart_id, baudrate=baud, tx=Pin(tx_pin), rx=Pin(rx_pin))
        time.sleep(0.2)  # allow UART to settle
        self.log_lines = []
        self.max_log_lines = 10

    def _send(self, payload: bytes):
        self.uart.write(payload)
        self.uart.write(_NEXTION_END)

    def cmd(self, text: str):
        self._send(text.encode("ascii"))

    def goto_page(self, page_num: int):
        self.cmd(f"page {page_num}")

    @staticmethod
    def _safe(s: str) -> str:
        # Avoid double quotes breaking the command
        return str(s).replace('"', "'")

    def set_text(self, obj: str, text: str):
        safe = self._safe(text)
        self._send(f'{obj}.txt="'.encode() + safe.encode() + b'"')

    def set_text_multiline(self, obj: str, lines):
        safe_lines = [self._safe(x) for x in lines]
        joined = "\r\n".join(safe_lines)  # real CRLF inside string
        self._send(f'{obj}.txt="'.encode() + joined.encode() + b'"')

    def clear_log(self):
        self.log_lines = []
        self.set_text("t1", "")

    def log(self, line: str):
        # Keep a rolling buffer
        line = self._safe(line)
        self.log_lines.append(line)
        if len(self.log_lines) > self.max_log_lines:
            self.log_lines = self.log_lines[-self.max_log_lines:]

        self.set_text_multiline("t1", self.log_lines)

    def status(self, heading_deg: float, text: str):
        # One clean status line
        self.set_text("t0", f"HDG {heading_deg:5.1f} | {self._safe(text)}")
