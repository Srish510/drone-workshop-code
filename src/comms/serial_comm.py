"""Serial communication handler for GCS ↔ Drone link."""

import threading
import serial

from utils.logging import setup_logger
from utils.message_parser import decode_message

logger = setup_logger("serial_comm")


class SerialComm:
    """Bidirectional serial communication with the GCS."""

    def __init__(self, port: str, baud_rate: int = 57600, timeout: float = 1.0):
        """Initialize serial connection parameters.

        Args:
            port: Serial port path (e.g. /dev/ttyUSB0).
            baud_rate: Communication baud rate.
            timeout: Read timeout in seconds.
        """
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.ser = None
        self._running = False
        self._recv_thread = None
        self._on_message = None

    def connect(self, on_message=None):
        """Open the serial port and start the receive loop.

        Args:
            on_message: Callback function(dict) for incoming messages.
        """
        self._on_message = on_message
        self.ser = serial.Serial(
            self.port, baudrate=self.baud_rate, timeout=self.timeout
        )
        self._running = True
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()
        logger.info("Serial connected on %s at %d baud", self.port, self.baud_rate)

    def _recv_loop(self):
        """Continuously read lines from the serial port."""
        while self._running:
            try:
                line = self.ser.readline()
                if line:
                    msg = decode_message(line)
                    if msg and self._on_message:
                        self._on_message(msg)
            except Exception as e:
                if self._running:
                    logger.error("Serial read error: %s", e)

    def send(self, data: bytes):
        """Send raw bytes over serial.

        Args:
            data: Bytes to transmit.
        """
        if self.ser and self.ser.is_open:
            self.ser.write(data)

    def close(self):
        """Stop the receive loop and close the serial port."""
        self._running = False
        if self._recv_thread:
            self._recv_thread.join(timeout=3)
        if self.ser and self.ser.is_open:
            self.ser.close()
        logger.info("Serial connection closed")
