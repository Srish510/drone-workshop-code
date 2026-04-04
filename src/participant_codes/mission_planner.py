import importlib
from utils.logging import setup_logger

logger = setup_logger("mission_planner")

class MissionPlanner:
    def __init__(self, name: str):
        self.name = name
        
    def start_mission(self, fc) -> bool:
        try:
            module = importlib.import_module(f"participant_codes.{self.name}")
            if hasattr(module, "mission_plan"):
                waypoints = module.mission_plan()
                for wp in waypoints:
                    x, y, alt = wp
                    fc.add_waypoint(x, y, alt)
                return True
            else:
                logger.error(f"Module {self.name} does not have a mission_plan function")
                return False
        except ImportError as e:    
            logger.error(f"Failed to import mission module {self.name}: {e}")
            return False