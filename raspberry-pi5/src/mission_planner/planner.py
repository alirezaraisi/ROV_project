from typing import Optional
from . import missions
import log

logger = log.getLogger(__name__)

home_lat: float = 0
home_lon: float = 0
home_alt: float = 0
current_missions: list[missions.Mission] = []
new_missions: list[missions.Mission] = []
new_missions_count = 0


def get_current_home() -> tuple[float, float, float]:
    return home_lat, home_lon, home_alt


def set_current_home(lat: float, lon: float, alt: float):
    global home_lat, home_lon, home_alt
    logger.info(
        "Setting current home position: lat=%.6f, lon=%.6f, alt=%.2f", lat, lon, alt)
    home_lat = lat
    home_lon = lon
    home_alt = alt


def get_missions_count():
    return len(current_missions)


def get_mission_item(seq: int) -> Optional[missions.Mission]:
    if 0 <= seq < len(current_missions):
        return current_missions[seq]
    logger.warning("Mission %d not found", seq)
    return None


def prepare_mission_download(mission_count: int):
    global new_missions_count, current_missions
    if mission_count == 0:
        logger.info("Current missions cleared.")
        new_missions_count = mission_count
        current_missions = []
        return

    logger.info("Preparing downloading %d new missions...", mission_count)
    new_missions_count = mission_count


def store_mission(new_mission: missions.Mission, seq: int) -> int:
    global current_missions, new_missions, new_missions_count

    if seq >= new_missions_count:
        logger.warning(
            "Attempted to store mission out of bounds: %s (%d)", new_mission.command, seq)
        return -1

    logger.debug("Storing mission...")
    new_missions.append(new_mission)

    if seq == new_missions_count - 1:
        logger.info("All new missions stored successfully.")
        current_missions = new_missions
        new_missions_count = 0
        new_missions = []

    return new_missions_count - len(new_missions)


def start_mission(start_mission: int, end_mission: int):
    logger.info("Starting missions from %d to %d", start_mission, end_mission)
    for i in range(start_mission, end_mission + 1):
        mission = get_mission_item(i)
        if mission:
            logger.info("Running mission %d: %s", i, mission)
        else:
            logger.warning("Mission %d not found", i)
