"""Disconnect failsafe – placeholder for GCS disconnect handling."""

from utils.logging import setup_logger

logger = setup_logger("failsafe.disconnect")


def on_gcs_disconnect(flight_controller):
    """Handle loss of GCS communication.

    Args:
        flight_controller: FlightController instance to command landing.
    """
    logger.warning("GCS connection lost – initiating emergency land")
    flight_controller.land()
