"""Battery drop failsafe – placeholder for battery monitoring logic."""

from utils.logging import setup_logger

logger = setup_logger("failsafe.battery_drop")


def check_battery(telemetry: dict, threshold: float = 20.0) -> bool:
    """Check if battery level is critically low.

    Args:
        telemetry: Current telemetry dictionary.
        threshold: Minimum acceptable battery percentage.

    Returns:
        True if battery is OK, False if critically low.
    """
    remaining = telemetry.get("battery_remaining", -1)
    if remaining < 0:
        return True  # Unknown, assume OK
    if remaining < threshold:
        logger.warning("Battery critically low: %d%%", remaining)
        return False
    return True
