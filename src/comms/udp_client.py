import socket
import threading

from utils.logging import setup_logger
from utils.message_parser import decode_message

logger = setup_logger("udp_client")

# Maximum UDP datagram size for our messages
MAX_UDP_SIZE = 65535


class UDPComm:
    def __init__(self, host: str = "0.0.0.0", port: int = 5000):
        self.host = host
        self.port = port
        self.sock = None
        self.gcs_addr = None
        self._running = False
        self._recv_thread = None
        self._on_message = None

    def connect(self, on_message=None):
        self._on_message = on_message
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.host, self.port))
        self.sock.settimeout(1.0)
        self._running = True
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()
        logger.info("UDP listening on %s:%d", self.host, self.port)

    def _recv_loop(self):
        while self._running:
            try:
                data, addr = self.sock.recvfrom(MAX_UDP_SIZE)
                if data:
                    self.gcs_addr = addr
                    
                    if data.startswith(b"IMG"):
                        if self._on_message:
                            self._on_message(data)
                            
                    else:
                        msg = decode_message(data)
                        if msg and self._on_message:
                            self._on_message(msg)
                            
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    logger.error("UDP receive error: %s", e)

    def send(self, data: bytes):
        if self.sock and self.gcs_addr:

            if data.startswith(b"VID") or data.startswith(b"IMG"):
                gcs_ip = self.gcs_addr[0]
                video_port = self.port + 50
                video_target_addr = (gcs_ip, video_port)
                
                self.sock.sendto(data, video_target_addr)
                
            else:
                self.sock.sendto(data, self.gcs_addr)

    def close(self):
        self._running = False
        if self._recv_thread:
            self._recv_thread.join(timeout=3)
        if self.sock:
            self.sock.close()
        logger.info("UDP socket closed")