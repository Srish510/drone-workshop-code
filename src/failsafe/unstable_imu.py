"""Unstable IMU failsafe – placeholder for IMU anomaly detection."""

from utils.logging import setup_logger

logger = setup_logger("failsafe.unstable_imu")


def check_imu(telemetry: dict, max_accel: float = 30.0) -> bool:
    """Check for excessive acceleration values indicating IMU instability.

    Args:
        telemetry: Current telemetry dictionary.
        max_accel: Maximum acceptable acceleration (m/s²).

    Returns:
        True if IMU readings are within normal bounds.
    """
    for axis in ("ax", "ay", "az"):
        val = abs(telemetry.get(axis, 0.0))
        if val > max_accel:
            logger.warning("IMU instability detected: %s=%.2f", axis, val)
            return False
    return True
