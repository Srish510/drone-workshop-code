import os
import threading
import time
import struct
import math

from utils.logging import setup_logger

logger = setup_logger("image_rx")

RECEIVED_IMAGES_DIR = os.path.join(
    os.path.dirname(os.path.dirname(__file__)), "received_images"
)


class ImageHandler:
    def __init__(self, camera_config: dict = None):
        self.camera_config = camera_config or {}
        self.device_index = self.camera_config.get("device_index", 0)
        self.width = self.camera_config.get("width", 640)
        self.height = self.camera_config.get("height", 480)
        self.jpeg_quality = self.camera_config.get("jpeg_quality", 50)
        self.fps = self.camera_config.get("fps", 15)
        self.stream_port = self.camera_config.get("stream_port", 5002)
        self.camera_type = self.camera_config.get("type", "usb")

        self._streaming = False
        self._stream_thread = None

        os.makedirs(RECEIVED_IMAGES_DIR, exist_ok=True)


    def receive_image(self, image_bytes: bytes): 
        
        if not image_bytes:
            logger.warning("Empty image data received")
            return
            
        try:
            
            filename = f"img_{int(time.time() * 1000)}.jpg"
            filepath = os.path.join(RECEIVED_IMAGES_DIR, filename)
            
            with open(filepath, "wb") as f:
                f.write(image_bytes)
                
            logger.info("Image saved: %s (%d bytes)", filepath, len(image_bytes))
        except Exception as e:
            logger.error("Failed to save image: %s", e)
            

    def start_stream(self, comm_link):
        if self._streaming:
            logger.warning("Stream already running")
            return
        self._streaming = True
        self._stream_thread = threading.Thread(
            target=self._stream_loop, args=(comm_link,), daemon=True
        )
        self._stream_thread.start()
        logger.info("Camera stream started")


    def _stream_loop(self, comm_link):
        if self.camera_type == "usb":
            self._usb_stream_loop(comm_link)
        elif self.camera_type == "rpi":
            self._rpi_stream_loop(comm_link)
        else:
            logger.error("Unknown camera type: %s", self.camera_type)
            self._streaming = False

    def _send_chunked_frame(self, comm_link, image_bytes: bytes):
        MAX_PAYLOAD_SIZE = 1024 
        total_size = len(image_bytes)
        total_chunks = math.ceil(total_size / MAX_PAYLOAD_SIZE)
        
        # Use frame_id instead of image_id for clarity
        frame_id = int(time.time() * 1000) % 256 
        
        try:
            for i in range(total_chunks):
                start = i * MAX_PAYLOAD_SIZE
                end = start + MAX_PAYLOAD_SIZE
                chunk_data = image_bytes[start:end]
                
                # CHANGE: Use b"VID" tag to mark this as a transient video frame
                header = struct.pack(">3sBHH", b"VID", frame_id, i, total_chunks)
                packet = header + chunk_data
                
                comm_link.send(packet)
                time.sleep(0.001) # Reduced from 0.002 to 0.001 for smoother video
        except Exception as e:
            logger.error("Failed to send chunked frame: %s", e)
            
            
    def _usb_stream_loop(self, comm_link):
        try:
            import cv2
        except ImportError:
            logger.error("opencv-python is not installed; cannot stream camera")
            self._streaming = False
            return

        cap = cv2.VideoCapture(self.device_index)
        if not cap.isOpened():
            logger.error("Failed to open camera device %d", self.device_index)
            self._streaming = False
            return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        interval = 1.0 / self.fps

        while self._streaming:
            ret, frame = cap.read()
            if not ret:
                logger.warning("Failed to capture frame")
                time.sleep(interval)
                continue

            encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            _, jpeg = cv2.imencode(".jpg", frame, encode_params)

            self._send_chunked_frame(comm_link, jpeg.tobytes())

            time.sleep(interval)

        cap.release()
        logger.info("USB camera stream stopped")
        
        
    def _rpi_stream_loop(self, comm_link):
        try:
            from picamera2 import Picamera2
            import cv2
        except ImportError:
            logger.error("picamera2 or opencv not installed")
            self._streaming = False
            return

        picam2 = Picamera2()

        config = picam2.create_video_configuration(
            main={"size": (self.width, self.height)}
        )
        picam2.configure(config)
        picam2.start()

        interval = 1.0 / self.fps

        while self._streaming:
            try:
                frame = picam2.capture_array()

                encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
                _, jpeg = cv2.imencode(".jpg", frame, encode_params)
                self._send_chunked_frame(comm_link, jpeg.tobytes())

            except Exception as e:
                logger.warning("Pi camera capture failed: %s", e)

            time.sleep(interval)

        picam2.stop()
        logger.info("RPI camera stream stopped")


    def stop_stream(self):
        self._streaming = False
        if self._stream_thread:
            self._stream_thread.join(timeout=5)
        logger.info("Camera stream stopped")