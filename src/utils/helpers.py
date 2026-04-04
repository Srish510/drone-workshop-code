import os
import yaml
import math

def load_config(filename: str) -> dict:
    config_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "config")
    filepath = os.path.join(config_dir, filename)
    with open(filepath, "r") as f:
        return yaml.safe_load(f)


def get_pixhawk_connection_string(config: dict) -> str:
    mode = config.get("mode", "sitl")
    return config[mode]["connection_string"]


def get_pixhawk_baud_rate(config: dict) -> int:
    mode = config.get("mode", "sitl")
    return config[mode]["baud_rate"]

def haversine_dist(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    #returns distance in meters
    R = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c                

def meters_to_latlon(lat0: float, lon0: float, dx: float, dy: float) -> float:
    R = 6371000.0  # earth radius in meters
    lat0_rad = math.radians(lat0)
    dlat = (dy / R) * (180.0 / math.pi)
    dlon = (dx / (R * math.cos(lat0_rad))) * (180.0 / math.pi)

    return lat0 + dlat, lon0 + dlon
