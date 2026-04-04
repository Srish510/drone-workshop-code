import struct
import threading
import time

from comms.serial_comm import SerialComm
from comms.udp_client import UDPComm
from comms.tcp_client import TCPComm
from utils.logging import setup_logger
from utils.message_parser import encode_message
from participant_codes.mission_planner import MissionPlanner

logger = setup_logger("comm_controller")


class CommController:
    def __init__(self, conn_config: dict, flight_controller, image_handler=None):
        self.conn_config = conn_config
        self.fc = flight_controller
        self.image_handler = image_handler
        self.comm = None
        self._telemetry_thread = None
        self._running = False
        self._image_buffers = {}
        

    def start(self):
        conn_type = self.conn_config.get("connection_type", "udp")
        logger.info("Starting GCS link via %s", conn_type)

        if conn_type == "serial":
            cfg = self.conn_config["serial"]
            self.comm = SerialComm(
                port=cfg["port"],
                baud_rate=cfg["baud_rate"],
                timeout=cfg.get("timeout", 1.0),
            )
        elif conn_type == "udp":
            cfg = self.conn_config["udp"]
            self.comm = UDPComm(host=cfg["host"], port=cfg["port"])
        elif conn_type == "tcp":
            cfg = self.conn_config["tcp"]
            self.comm = TCPComm(host=cfg["host"], port=cfg["port"])
        else:
            raise ValueError(f"Unknown connection type: {conn_type}")

        self.comm.connect(on_message=self._handle_message)
        self._running = True
        self._telemetry_thread = threading.Thread(
            target=self._telemetry_loop, daemon=True
        )
        self._telemetry_thread.start()


    def stop(self):
        self._running = False
        if self._telemetry_thread:
            self._telemetry_thread.join(timeout=5)
        if self.comm:
            self.comm.close()
        logger.info("CommController stopped")


    def _handle_message(self, msg):

        self.image_handler.start_stream(self.comm)

        if isinstance(msg, bytes) and msg.startswith(b"IMG"):
            self._handle_image(msg)
            return
        
        if isinstance(msg, dict):
            msg_type = msg.get("type")
            if msg_type == "command":
                command = msg.get("command", {})
                params = msg.get("params", {})
                self._handle_command(command, params)
            elif msg_type == "camera":
                self._handle_camera(msg)
            else:
                logger.warning("Unknown message type: %s", msg_type)
            return
    
        logger.warning("Received message in unknown format: %s", msg)   


    def _handle_command(self, command: str, params: dict):

        logger.info("Received command: %s", command)

        if command == "arm":
            success = self.fc.arm()
            self._send_log("info" if success else "error",
                           f"Arm {'succeeded' if success else 'failed'}")
        elif command == "takeoff":
            altitude = params.get("altitude")
            success = self.fc.takeoff(altitude)
            self._send_log("info" if success else "error",
                           f"Takeoff {'succeeded' if success else 'failed'}")
        elif command == "start_mission":
            success = MissionPlanner(params.get("name")).start_mission(self.fc)
            self._send_log("info" if success else "error",
                           f"Mission {params.get("name")} start {'succeeded' if success else 'failed'}")
        elif command == "land":
            success = self.fc.land()
            self._send_log("info" if success else "error",
                           f"Land {'succeeded' if success else 'failed'}")
        else:
            logger.warning("Unknown command action: %s", command)
            self._send_log("warning", f"Unknown command: {command}")


    def _handle_image(self, data: bytes):
        header_format = ">3sBHH"
        header_size = struct.calcsize(header_format)
        
        # Ensure the packet is large enough to contain our header
        if len(data) < header_size:
            logger.warning("Received undersized image chunk, discarding.")
            return
            
        # Unpack the metadata
        _, image_id, chunk_index, total_chunks = struct.unpack(header_format, data[:header_size])
        chunk_data = data[header_size:]
        
        # Initialize the buffer for this specific image ID if it doesn't exist
        if image_id not in self._image_buffers:
            self._image_buffers[image_id] = {}
            
        # Store the chunk
        self._image_buffers[image_id][chunk_index] = chunk_data
        
        # Check if we have received all chunks
        if len(self._image_buffers[image_id]) == total_chunks:
            logger.info(f"All chunks received for image {image_id}. Reassembling...")
            
            # Reassemble the bytes in order
            complete_image_bytes = b"".join(
                self._image_buffers[image_id][i] for i in range(total_chunks)
            )
            
            # Clean up the buffer
            del self._image_buffers[image_id]
            
            # Pass the complete bytes to the image handler
            if self.image_handler:
                self.image_handler.receive_image(complete_image_bytes)
                self._send_log("info", f"Image {image_id} successfully reassembled.")
            else:
                logger.warning("Image fully received but no image handler configured.")


    def _handle_camera(self, data: dict):
        if not self.image_handler:
            logger.warning("Camera command received but no image handler configured")
            return

        action = data.get("action", "").lower()
        if action == "start":
            self.image_handler.start_stream(self.comm)
            self._send_log("info", "Camera stream started")
        elif action == "stop":
            self.image_handler.stop_stream()
            self._send_log("info", "Camera stream stopped")


    def _telemetry_loop(self):
        rate = self.fc.telemetry_rate_hz if self.fc else 10
        interval = 1.0 / rate
        while self._running:
            if self.fc and self.comm:
                telemetry = self.fc.get_telemetry()
                msg = encode_message("telemetry", telemetry)
                self.comm.send(msg)
            time.sleep(interval)


    def _send_log(self, level: str, message: str):
        if self.comm:
            msg = encode_message("log", {"level": level, "message": message})
            self.comm.send(msg)
