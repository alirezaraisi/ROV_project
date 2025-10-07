from . import mavlink

from enum import Enum
import copy

__all__ = [
    'mode_string_v10',
    'add_message',
    'set_close_on_exec',
    'VehicleModes',
    'ButtonFunctions',
    'mavfile_state',
    'param_state'
]


def mode_string_v10(msg):
    '''mode string for 1.0 protocol, from heartbeat'''
    return f"Mode({VehicleModes(msg.custom_mode).name})"


def add_message(messages, mtype, msg):
    '''add a msg to array of messages, taking account of instance messages'''
    if msg._instance_field is None or getattr(msg, msg._instance_field, None) is None:
        # simple case, no instance field
        messages[mtype] = msg
        return
    instance_value = getattr(msg, msg._instance_field)
    if not mtype in messages:
        messages[mtype] = copy.copy(msg)
        messages[mtype]._instances = {}
        messages[mtype]._instances[instance_value] = msg
        messages["%s[%s]" % (mtype, str(instance_value))] = copy.copy(msg)
        return
    messages[mtype]._instances[instance_value] = msg
    prev_instances = messages[mtype]._instances
    messages[mtype] = copy.copy(msg)
    messages[mtype]._instances = prev_instances
    messages["%s[%s]" % (mtype, str(instance_value))] = copy.copy(msg)


def set_close_on_exec(fd):
    '''set the close on exec flag on a file descriptor. Ignore exceptions'''
    try:
        import fcntl
        flags = fcntl.fcntl(fd, fcntl.F_GETFD)
        flags |= fcntl.FD_CLOEXEC
        fcntl.fcntl(fd, fcntl.F_SETFD, flags)
    except Exception:
        pass


class VehicleModes(Enum):
    # Mode not set by vehicle yet
    PRE_FLIGHT = -1
    # Manual angle with manual depth/throttle
    STABILIZE = 0
    # Manual body-frame angular rate with manual depth/throttle
    ACRO = 1
    # Manual angle with automatic depth/throttle
    ALT_HOLD = 2
    # Fully automatic waypoint control using mission commands
    AUTO = 3
    # Fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    GUIDED = 4
    # Automatic circular flight with automatic throttle
    CIRCLE = 7
    # Automatically return to surface, pilot maintains horizontal control
    SURFACE = 9
    # Automatic position hold with manual override, with automatic throttle
    POSHOLD = 16
    # Pass-through input with no stabilization
    MANUAL = 19
    # Automatically detect motors orientation
    MOTOR_DETECT = 20
    # Manual angle with automatic depth/throttle (from rangefinder altitude)
    SURFTRAK = 21


# fmt: off
class ButtonFunctions(Enum):
    KNone                 = 0            # < disabled
    Shift                 = 1            # < "shift" buttons to allow more functions
    ArmToggle             = 2            # < arm/disarm vehicle toggle
    Arm                   = 3            # < arm vehicle
    Disarm                = 4            # < disarm vehicle

    ModeManual            = 5            # < enter enter manual mode
    ModeStabilize         = 6            # < enter stabilize mode
    ModeDepthHold         = 7            # < enter depth hold mode
    ModePoshold           = 8            # < enter poshold mode
    ModeAuto              = 9            # < enter auto mode
    ModeCircle            = 10           # < enter circle mode
    ModeGuided            = 11           # < enter guided mode
    ModeAcro              = 12           # < enter acro mode
    ModeSurftrak          = 13           # < enter surftrak mode

    # 14-20 reserved for future mode functions
    MountCenter           = 21           # < move mount to center
    MountTiltUp           = 22           # < tilt mount up
    MountTiltDown         = 23           # < tilt mount down
    CameraTrigger         = 24           # < trigger camera shutter
    CameraSourceToggle    = 25           # < toggle camera source
    MountPanRight         = 26           # < pan mount right
    MountPanLeft          = 27           # < pan mount left
    # 26-30 reserved for future camera functions
    Lights1Cycle          = 31           # < lights 1 cycle
    Lights1Brighter       = 32           # < lights 1 up
    Lights1Dimmer         = 33           # < lights 1 down
    Lights2Cycle          = 34           # < lights 2 cycle
    Lights2Brighter       = 35           # < lights 2 up
    Lights2Dimmer         = 36           # < lights 2 down
    # 37-40 reserved for future light functions
    GainToggle            = 41           # < toggle different gain settings
    GainInc               = 42           # < increase control gain
    GainDec               = 43           # < decrease control gain
    TrimRollInc           = 44           # < increase roll trim
    TrimRollDec           = 45           # < decrease roll trim
    TrimPitchInc          = 46           # < increase pitch trim
    TrimPitchDec          = 47           # < decrease pitch trim
    InputHoldSet          = 48           # < toggle input hold (trim to current controls)
    RollPitchToggle       = 49           # < adjust roll/pitch input instead of forward/lateral

    # 50 reserved for future function

    Relay1On              = 51           # < trigger relay on
    Relay1Off             = 52           # < trigger relay off
    Relay1Toggle          = 53           # < trigger relay toggle
    Relay2On              = 54           # < trigger relay on
    Relay2Off             = 55           # < trigger relay off
    Relay2Toggle          = 56           # < trigger relay toggle
    Relay3On              = 57           # < trigger relay on
    Relay3Off             = 58           # < trigger relay off
    Relay3Toggle          = 59           # < trigger relay toggle

    # 60 reserved for future function
    Servo1Inc             = 61           # < increase servo output
    Servo1Dec             = 62           # < decrease servo output
    Servo1Min             = 63           # < center servo
    Servo1Max             = 64           # < set servo output to minimum (SERVOn_MIN)
    Servo1Center          = 65           # < set servo output to maximum (SERVOn_MAX)

    Servo2Inc             = 66
    Servo2Dec             = 67
    Servo2Min             = 68
    Servo2Max             = 69
    Servo2Center          = 70

    Servo3Inc             = 71
    Servo3Dec             = 72
    Servo3Min             = 73
    Servo3Max             = 74
    Servo3Center          = 75

    Servo1MinMomentary    = 76          # < set servo output to minimum (SERVOn_MIN) until released
    Servo1MaxMomentary    = 77          # < set servo output to minimum (SERVOn_MAX) until released
    Servo1MinToggle       = 78          # < toggle servo output btwn trim (SERVOn_TRIM) and min (SERVOn_MIN)
    Servo1MaxToggle       = 79          # < toggle servo output btwn trim (SERVOn_TRIM) and max (SERVOn_MAX)

    Servo2MinMomentary    = 80
    Servo2MaxMomentary    = 81
    Servo2MinToggle       = 82
    Servo2MaxToggle       = 83

    Servo3MinMomentary    = 84
    Servo3MaxMomentary    = 85
    Servo3MinToggle       = 86
    Servo3MaxToggle       = 87

    # 88-90 reserved for future functions
    Custom1               = 91           # < custom user button 1
    Custom2               = 92           # < custom user button 2
    Custom3               = 93           # < custom user button 3
    Custom4               = 94           # < custom user button 4
    Custom5               = 95           # < custom user button 5
    Custom6               = 96           # < custom user button 6
    # 97-100 reserved for future functions
    Relay4On              = 101           # < trigger relay on
    Relay4Off             = 102           # < trigger relay off
    Relay4Toggle          = 103           # < trigger relay toggle

    Relay1Momentary       = 104           # < relay toggle when button is pushed and again when released
    Relay2Momentary       = 105
    Relay3Momentary       = 106
    Relay4Momentary       = 107

    Script1               = 108
    Script2               = 109
    Script3               = 110
    Script4               = 111

    Servo4Min             = 112
    Servo4Max             = 113
    Servo4Center          = 114
    Servo4Inc             = 115
    Servo4Dec             = 116
    Servo4MinMomentary    = 117
    Servo4MaxMomentary    = 118
    Servo4MinToggle       = 119
    Servo4MaxToggle       = 120

    Servo5Min             = 121
    Servo5Max             = 122
    Servo5Center          = 123
    Servo5Inc             = 124
    Servo5Dec             = 125
    Servo5MinMomentary    = 126
    Servo5MaxMomentary    = 127
    Servo5MinToggle       = 128
    Servo5MaxToggle       = 129

    Servo6Min             = 130
    Servo6Max             = 131
    Servo6Center          = 132
    Servo6Inc             = 133
    Servo6Dec             = 134
    Servo6MinMomentary    = 135
    Servo6MaxMomentary    = 136
    Servo6MinToggle       = 137
    Servo6MaxToggle       = 138

    NrBtnFunctions        = 139           # < This must be the last enum value (only add new values _before_ this one)
# fmt: on


class mavfile_state(object):
    '''state for a particular system id'''

    def __init__(self):
        self.messages = {'MAV': self}
        self.flightmode = "UNKNOWN"
        self.vehicle_type = "UNKNOWN"
        self.mav_type = mavlink.MAV_TYPE_FIXED_WING
        self.mav_autopilot = mavlink.MAV_AUTOPILOT_GENERIC
        self.base_mode = 0
        self.armed = False  # canonical arm state for the vehicle as a whole

        if float(mavlink.WIRE_PROTOCOL_VERSION) >= 1:
            try:
                self.messages['HOME'] = mavlink.MAVLinKGpsRawIntMessage(
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
            except AttributeError:
                # may be using a minimal dialect
                pass
            try:
                mavlink.MAVLinKWaypointMessage = mavlink.MAVLinKMissionItemMessage
            except AttributeError:
                # may be using a minimal dialect
                pass
        else:
            try:
                self.messages['HOME'] = mavlink.MAVLinKGpsRawMessage(
                    0, 0, 0, 0, 0, 0, 0, 0, 0)
            except AttributeError:
                # may be using a minimal dialect
                pass


class param_state(object):
    '''state for a particular system id/component id pair'''

    def __init__(self):
        self.params = {}
