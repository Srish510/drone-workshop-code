import time
import threading
import math
from pymavlink import mavutil

from utils.logging import setup_logger
from utils.helpers import meters_to_latlon, haversine_dist

logger = setup_logger("flight_controller")

# MAVLink flight modes
GUIDED_NOGPS = 20
GUIDED = 4
LAND = 9

class FlightController:
    def __init__(self, connection_string: str, baud_rate: int = 57600,
                 flight_config: dict = None):
        
        self.connection_string = connection_string
        self.baud_rate = baud_rate
        self.flight_config = flight_config or {}
        self.takeoff_altitude = self.flight_config.get("takeoff_altitude", 2.0)
        self.arm_timeout = self.flight_config.get("arm_timeout", 30)
        self.takeoff_timeout = self.flight_config.get("takeoff_timeout", 30)
        self.telemetry_rate_hz = self.flight_config.get("telemetry_rate_hz", 10)

        self.connection = None
        self.is_armed = False
        self.telemetry = {
            "flight_mode" : "UNKNOWN",
            "altitude": 0.0,
            "battery": -1,
            "lat": 0.0,
            "lon": 0.0,
            "vx": 0.0, "vy": 0.0, "vz": 0.0,
            "ax": 0.0, "ay": 0.0, "az": 0.0,
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            "armed": False
        }
        
        # Threading and synchronization
        self._telemetry_lock = threading.Lock()
        self._state_lock = threading.Lock() 
        self._telemetry_thread = None
        self._control_thread = None         
        self._running = False
        self._ack_event = threading.Event()
        self._latest_ack = None
        
        # Navigation Data
        self.lat0 = 0.0
        self.lon0 = 0.0
        self.waypoints = []
        
        # Control Loop State Tracking
        self.target_count = -1
        self.checked = False
        self.landing = False
        self.tgt_x = 0.0
        self.tgt_y = 0.0
        self.tgt_alt = 0.0

    def connect(self):
        logger.info("Connecting to Pixhawk at %s (baud=%d)",
                     self.connection_string, self.baud_rate)
        self.connection = mavutil.mavlink_connection(
            self.connection_string, baud=self.baud_rate
        )
        logger.info("Waiting for heartbeat...")
        self.connection.wait_heartbeat()
        logger.info("Heartbeat received (system %d, component %d)",
                     self.connection.target_system,
                     self.connection.target_component)

        self._request_data_streams()
        self._running = True
        
        # Start telemetry thread
        self._telemetry_thread = threading.Thread(
            target=self._telemetry_loop, daemon=True
        )
        self._telemetry_thread.start()
        
        # Start control loop thread
        self._control_thread = threading.Thread(
            target=self._control_loop, daemon=True
        )
        self._control_thread.start()

    def disconnect(self):
        self._running = False
        if self._telemetry_thread:
            self._telemetry_thread.join(timeout=5)
        if self._control_thread:
            self._control_thread.join(timeout=5)
        if self.connection:
            self.connection.close()
            logger.info("Pixhawk connection closed")

    def _request_data_streams(self):
        self.connection.mav.request_data_stream_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            self.telemetry_rate_hz,
            1,
        )
        logger.info("Requested data streams at %d Hz", self.telemetry_rate_hz)

    def _telemetry_loop(self):
        while self._running:
            msg = self.connection.recv_match(blocking=True, timeout=1)
            if msg is None:
                continue
            msg_type = msg.get_type()
            with self._telemetry_lock:
                if msg_type == "GLOBAL_POSITION_INT":
                    self.telemetry["lat"] = msg.lat / 1e7
                    self.telemetry["lon"] = msg.lon / 1e7
                    self.telemetry["altitude"] = msg.relative_alt / 1000.0
                elif msg_type == "LOCAL_POSITION_NED":
                    self.telemetry["altitude"] = -msg.z
                    self.telemetry["vx"] = msg.vx
                    self.telemetry["vy"] = msg.vy
                    self.telemetry["vz"] = msg.vz
                elif msg_type in ("SCALED_IMU", "SCALED_IMU2"):
                    self.telemetry["ax"] = msg.xacc / 1000.0
                    self.telemetry["ay"] = msg.yacc / 1000.0
                    self.telemetry["az"] = msg.zacc / 1000.0
                elif msg_type == "ATTITUDE":
                    self.telemetry["roll"] = math.degrees(msg.roll)
                    self.telemetry["pitch"] = math.degrees(msg.pitch)
                    self.telemetry["yaw"] = math.degrees(msg.yaw)
                elif msg_type == "SYS_STATUS":
                    self.telemetry["battery"] = msg.battery_remaining
                elif msg_type == "HEARTBEAT":
                    self.is_armed = (msg.base_mode &
                                     mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                    self.telemetry["armed"] = self.is_armed
                elif msg_type == "COMMAND_ACK":
                    self._latest_ack = msg
                    self._ack_event.set()
                if self.connection.flightmode:
                    self.telemetry["flight_mode"] = self.connection.flightmode

    def get_telemetry(self) -> dict:
        with self._telemetry_lock:
            return dict(self.telemetry)

    def set_mode(self, mode_id: int) -> bool:
        self.connection.mav.set_mode_send(
            self.connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        ack = self.connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
        if ack:
            logger.info("Mode set to %d (result=%d)", mode_id, ack.result)
            return ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
        logger.warning("No ACK received for set_mode(%d)", mode_id)
        return False

    def arm(self) -> bool:
        logger.info("Setting GUIDED mode...")
        self.set_mode(GUIDED)

        with self._telemetry_lock:
            is_armed = self.is_armed 
            self.lat0 = self.telemetry.get("lat", 0.0)
            self.lon0 = self.telemetry.get("lon", 0.0)
            
        with self._state_lock:
            self.waypoints = []  
            self.waypoints.append((0.0, 0.0, self.takeoff_altitude))
            
            # Reset control loop state on arm
            self.target_count = -1
            self.checked = False
            self.landing = False
            self.tgt_x = 0.0
            self.tgt_y = 0.0
            self.tgt_alt = self.takeoff_altitude

        self._ack_event.clear() 
        
        if is_armed:
            logger.info("Sending disarm command...")
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0, 0, 0, 0, 0, 0,
            )
        else:
            logger.info("Sending arm command...")
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0,
            )

        if self._ack_event.wait(self.arm_timeout):
            ack = self._latest_ack
            if ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    with self._telemetry_lock:
                        self.is_armed = not self.is_armed
                        logger.info(f"Drone {'disarmed' if not self.is_armed else 'armed'} successfully")
                    return True
                logger.error("Arm command rejected (result=%d)", ack.result)
                return False
                
        logger.error("Arm command timed out")
        return False

    def takeoff(self, altitude: float = 3) -> bool:
        alt = altitude if altitude is not None else self.takeoff_altitude
        if not self.is_armed:
            logger.error("Cannot take off: drone is not armed")
            return False

        self._ack_event.clear()

        logger.info("Sending takeoff command to %.1f m", alt)
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, alt,
        )

        if self._ack_event.wait(10):
            ack = self._latest_ack
            if ack.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    logger.info("Takeoff command accepted")
                    return True
                logger.error("Takeoff rejected (result=%d)", ack.result)
                return False
                
        logger.warning("No ACK received for takeoff")
        return False

    def land(self) -> bool:
        logger.info("Sending land command...")
        
        self.landing = True
        self.set_mode(LAND)

        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0,
        )

        ack = self.connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=10)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                logger.info("Land command accepted")
                self.is_armed = False
                return True
            logger.error("Land rejected (result=%d)", ack.result)
            return False
        logger.warning("No ACK received for land")
        return False
    
    def go_to_coordinates(self, x: float, y: float, alt: float) -> bool:
        logger.info("Going to coordinates: x=%.6f, y=%.6f, z=%.1f", x, y, alt)
        lat, lon = meters_to_latlon(self.lat0, self.lon0, x, y)
        self.connection.mav.set_position_target_global_int_send(
            0,
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b110111111000,  
            int(lat * 1e7),       
            int(lon * 1e7),       
            alt,                  
            0, 0, 0,             
            0, 0, 0,             
            0, 0                  
        )
        ack = self.connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=10)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                logger.info("Go to coordinates command accepted")
                return True
            logger.error("Go to coordinates rejected (result=%d)", ack.result)
            return False
        logger.warning("No ACK received for go_to_coordinates")
        return False
    
    def add_waypoint(self, x: float, y: float, alt: float):
        logger.info("Adding waypoint: x=%.6f, y=%.6f, z=%.1f", x, y, alt)
        with self._state_lock:
            self.waypoints.append((x, y, alt))
        
        
    def _control_loop(self):
        HORIZ_TOL = 2.0   # meters
        VERT_TOL  = 0.5   # meters

        while self._running:
            #Take ATOMIC SNAPSHOT
            with self._telemetry_lock:
                cur_lat = self.telemetry.get("lat", 0.0)
                cur_lon = self.telemetry.get("lon", 0.0)
                cur_alt = self.telemetry.get("altitude", 0.0)

            with self._state_lock:
                tgt_x = self.tgt_x
                tgt_y = self.tgt_y
                tgt_alt = self.tgt_alt
                count   = self.target_count
                check   = self.checked
                landing = self.landing
                targets = list(self.waypoints)

            #Compute distance OUTSIDE the lock
            #Convert local target coordinates to global for accurate Haversine check
            tgt_lat, tgt_lon = meters_to_latlon(self.lat0, self.lon0, tgt_x, tgt_y)
            d_h = haversine_dist(cur_lat, cur_lon, tgt_lat, tgt_lon)
            d_v = abs(cur_alt - tgt_alt)

            next_target = None
            need_mode_change = False


            if not landing:
                # Reached takeoff altitude
                if count == -1 and cur_alt >= self.takeoff_altitude - VERT_TOL:
                    count = 0
                    logger.info("Takeoff altitude reached.")

                # Proceed to first waypoint
                if count == 0:
                    count = 1
                    if count < len(targets):
                        next_target = targets[count]
                        self.go_to_coordinates(*next_target)

                # Arrived at waypoint
                elif d_h <= HORIZ_TOL and d_v <= VERT_TOL and count > 0 and not check:
                    logger.info(f"Reached waypoint {count}")
                    need_mode_change = True
                    check = True
                    count += 1
                    if count < len(targets):
                        next_target = targets[count]
                        
                # En route to next target
                elif (d_h > HORIZ_TOL or d_v > VERT_TOL) and count > 0: 
                    check = False

            #Apply state changes ATOMICALLY
            with self._state_lock:
                self.target_count = count
                self.checked = check
                if next_target:
                    self.tgt_x, self.tgt_y, self.tgt_alt = next_target

            #Only now run blocking operations
            if need_mode_change:
                time.sleep(5)
                if not self.landing:
                    self.set_mode(GUIDED)
                    if next_target:
                        self.go_to_coordinates(*next_target)

            time.sleep(0.2)