from machine import UART, Pin
import time

_NEXTION_END = b"\xff\xff\xff"


class Nextion:

    def __init__(self, uart_id=0, tx_pin=16, rx_pin=17, baud=9600):
        self.uart = UART(
            uart_id,
            baudrate=baud,
            tx=Pin(tx_pin),
            rx=Pin(rx_pin)
        )

        time.sleep(0.2)

        self.log_lines = []
        self.max_log_lines = 10

    def _send(self, payload: bytes):
        self.uart.write(payload)
        self.uart.write(_NEXTION_END)

    def cmd(self, text: str):
        self._send(text.encode("ascii"))

    def goto_page(self, page_num: int):
        self.cmd("page {}".format(page_num))

    @staticmethod
    def _safe(s: str) -> str:
        return str(s).replace('"', "'")

    def set_text(self, obj: str, text: str):
        safe = self._safe(text)
        self._send(
            f'{obj}.txt="'.encode() +
            safe.encode() +
            b'"'
        )

    def set_text_multiline(self, obj: str, lines):
        safe_lines = [self._safe(x) for x in lines]
        joined = "\r\n".join(safe_lines)
        self._send(
            f'{obj}.txt="'.encode() +
            joined.encode() +
            b'"'
        )

    def clear_log(self):
        self.log_lines = []
        self.set_text("t1", "")

    def log(self, line: str):
        line = self._safe(line)
        self.log_lines.append(line)

        if len(self.log_lines) > self.max_log_lines:
            self.log_lines = self.log_lines[-self.max_log_lines:]

        self.set_text_multiline("t1", self.log_lines)

    def status(self, heading_deg: float, text: str):
        msg = "HDG {:5.1f} | {}".format(
            heading_deg,
            self._safe(text)
        )
        self.set_text("t0", msg)
