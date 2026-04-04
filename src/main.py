import signal
import sys
import time

from utils.helpers import load_config, get_pixhawk_connection_string, get_pixhawk_baud_rate
from utils.logging import setup_logger
from controllers.flight_controller import FlightController
from controllers.comm_controller import CommController
from search.image_rx import ImageHandler

logger = setup_logger("main")


def main():
    logger.info("RPi Drone System Starting")

    # Load configuration
    pixhawk_config = load_config("pixhawk_config.yaml")
    conn_config = load_config("connections.yaml")

    # Initialize flight controller
    connection_string = get_pixhawk_connection_string(pixhawk_config)
    baud_rate = get_pixhawk_baud_rate(pixhawk_config)
    flight_config = pixhawk_config.get("flight", {})

    fc = FlightController(
        connection_string=connection_string,
        baud_rate=baud_rate,
        flight_config=flight_config,
    )

    # Initialize image handler
    camera_config = conn_config.get("camera", {})
    image_handler = ImageHandler(camera_config=camera_config)

    # Initialize communication controller
    comm = CommController(
        conn_config=conn_config,
        flight_controller=fc,
        image_handler=image_handler,
    )

    # Graceful shutdown handler
    def shutdown(signum, frame):
        logger.info("Shutdown signal received")
        comm.stop()
        fc.disconnect()
        sys.exit(0)


    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # Connect to Pixhawk
    try:
        fc.connect()
    except Exception as e:
        logger.error("Failed to connect to Pixhawk: %s", e)
        sys.exit(1)

    # Start GCS communication
    try:
        comm.start()
    except Exception as e:
        logger.error("Failed to start GCS communication: %s", e)
        fc.disconnect()
        sys.exit(1)

    logger.info("RPi Drone System Ready")
    logger.info("Waiting for GCS commands...")

    # Main loop – keep alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received")
    finally:
        comm.stop()
        fc.disconnect()
        logger.info("RPi Drone System Stopped")


if __name__ == "__main__":
    main()