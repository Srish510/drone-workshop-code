"""TCP communication handler for GCS ↔ Drone link."""

import socket
import threading

from utils.logging import setup_logger
from utils.message_parser import decode_message

logger = setup_logger("tcp_client")

BUFFER_SIZE = 65535


class TCPComm:
    """Bidirectional TCP communication with the GCS.

    Acts as a TCP server: listens for a single GCS client connection.
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 5001):
        """Initialize TCP server parameters.

        Args:
            host: Bind address for listening.
            port: TCP port number.
        """
        self.host = host
        self.port = port
        self.server_sock = None
        self.client_sock = None
        self._running = False
        self._accept_thread = None
        self._recv_thread = None
        self._on_message = None

    def connect(self, on_message=None):
        """Start the TCP server and wait for a GCS connection.

        Args:
            on_message: Callback function(dict) for incoming messages.
        """
        self._on_message = on_message
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind((self.host, self.port))
        self.server_sock.listen(1)
        self.server_sock.settimeout(1.0)
        self._running = True
        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._accept_thread.start()
        logger.info("TCP server listening on %s:%d", self.host, self.port)

    def _accept_loop(self):
        """Accept a single GCS client connection."""
        while self._running:
            try:
                client, addr = self.server_sock.accept()
                logger.info("GCS connected from %s:%d", addr[0], addr[1])
                self.client_sock = client
                self.client_sock.settimeout(1.0)
                self._recv_thread = threading.Thread(
                    target=self._recv_loop, daemon=True
                )
                self._recv_thread.start()
                return
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    logger.error("TCP accept error: %s", e)

    def _recv_loop(self):
        """Continuously receive data from the connected GCS client."""
        buffer = b""
        while self._running and self.client_sock:
            try:
                data = self.client_sock.recv(BUFFER_SIZE)
                if not data:
                    logger.warning("GCS disconnected")
                    break
                buffer += data
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    msg = decode_message(line + b"\n")
                    if msg and self._on_message:
                        self._on_message(msg)
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    logger.error("TCP receive error: %s", e)
                break

    def send(self, data: bytes):
        """Send raw bytes to the connected GCS client.

        Args:
            data: Bytes to transmit.
        """
        if self.client_sock:
            try:
                self.client_sock.sendall(data)
            except Exception as e:
                logger.error("TCP send error: %s", e)

    def close(self):
        """Stop all loops and close sockets."""
        self._running = False
        if self._accept_thread:
            self._accept_thread.join(timeout=3)
        if self._recv_thread:
            self._recv_thread.join(timeout=3)
        if self.client_sock:
            self.client_sock.close()
        if self.server_sock:
            self.server_sock.close()
        logger.info("TCP server closed")
