from api.mavlink import mavlink
from abc import ABC, abstractmethod
import log

logger = log.getLogger(__name__)


class Mission(ABC):
    def __init__(self, x: float, y: float, z: float, frame: int, current: int, autocontinue: int, command: int):
        self.x = x
        self.y = y
        self.z = z
        self.frame = frame
        self.current = current
        self.autocontinue = autocontinue
        self.command = command

    @staticmethod
    @abstractmethod
    def from_message(msg: mavlink.MAVLink_mission_item_int_message):
        pass

    @abstractmethod
    def to_dict(self) -> dict:
        pass

    @abstractmethod
    def __repr__(self) -> str:
        pass


class MissionWayPoint(Mission):
    def __init__(self, hold, radius, passby, yaw, x: float, y: float, z: float, frame: int, current: int, autocontinue: int, command: int):
        super().__init__(x, y, z, frame, current, autocontinue, command)
        self.hold = hold
        self.radius = radius
        self.passby = passby
        self.yaw = yaw

    @staticmethod
    def from_message(msg: mavlink.MAVLink_mission_item_int_message):
        return MissionWayPoint(
            hold=msg.param1,
            radius=msg.param2,
            passby=msg.param3,
            yaw=msg.param4,
            x=msg.x / 10e7,
            y=msg.y / 10e7,
            z=msg.z,
            frame=msg.frame,
            current=msg.current,
            autocontinue=msg.autocontinue,
            command=msg.command
        )

    def to_dict(self) -> dict:
        return {
            'param1': self.hold,
            'param2': self.radius,
            'param3': self.passby,
            'param4': self.yaw,
            'x': int(self.x * 10e7),
            'y': int(self.y * 10e7),
            'z': self.z,
            'frame': self.frame,
            'current': self.current,
            'autocontinue': self.autocontinue,
            'command': self.command
        }

    def __repr__(self) -> str:
        return f"MissionWayPoint(x={self.x}, y={self.y}, z={self.z}, frame={self.frame}, current={self.current}, autocontinue={self.autocontinue}, command={self.command}, hold={self.hold}, radius={self.radius}, passby={self.passby}, yaw={self.yaw})"


def from_message(msg: mavlink.MAVLink_mission_item_int_message) -> Mission | None:
    logger.info(f"{msg.seq}: {msg.x}, {msg.y}, {msg.z}")
    if msg.command == mavlink.MAV_CMD_NAV_WAYPOINT:
        return MissionWayPoint.from_message(msg)

    logger.warning(f"Unknown mission command type: {msg.command}")
    return None
