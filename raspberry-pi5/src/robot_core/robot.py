import time
import log
from api.mavlink import VehicleModes
from hardware_interface import communication
from hardware_interface import motors
from hardware_interface import sensors
# from hardware_interface import camera
from hardware_interface import gnss
from hardware_interface import servo
from config import LOG_REPEAT_INTERVAL
from . import control, telemetry

#changed
from .filter_madgwick import IMUSensor
from . import pid


latest_joystick = "5000000500000"  # ????? ???? ????
mode = "MANUAL"
button = "0"
direction = "5"
change_pitch_depth = "pitch"

gear_yaw = 0.25
gear_depth =0.25

error_atuo = 0.0
#    The main class representing the ROV.
#    Manages state, hardware interfaces, and high-level operations.


logger = log.getLogger(__name__)
logger.info("Initializing ROV...")



# ---mode---


# --- State Variables ---
is_armed = False
custom_mode = VehicleModes.MANUAL  # Default mode

roll = 0.0
pitch = 0.0
yaw = 0.0

esp_depth = 0.0
esp_battery= 0
lat = 31.839417
lon = 54.358292
alt = 10.0
# Target normalized (-1 to 1)
current_target_movement = {
    "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
current_thruster_outputs: list[float] = []  # Actual values sent to thrusters


telemetry_data = {
    "depth": 0.0,
    "temperature": 0.0,
    #"imu": {},  # e.g., {'ax': 0, 'ay': 0, 'az': 0, ...}
    #"battery_voltage": 0.0,
    #"camera_pan": 0,
    #"camera_tilt": 0,
}


last_telemetry_update = time.time()
last_telemetry_warning = time.monotonic()

# --- Hardware Interfaces ---
# try:
#     # It's often better to pass the port and baud from a central config
#     if not communication.is_connected():
#         raise ConnectionError(
#             f"Failed to connect to ROV on {ROV_SERIAL_PORT}")

#     # camera_video = cv2.VideoCapture(0) # For actual video, if applicable

# except ConnectionError as e:
#     logger.error(f"HAL Initialization Error: {e}")
#     # Depending on desired behavior, you might re-raise, or allow offline mode
#     raise  # Re-raise to indicate critical failure
# except ImportError as e:
#     logger.error(
#         f"Import error, ensure pyserial is installed or hardware_interface modules are correct: {e}")
#     raise
# except Exception as e:
#     logger.error(
#         f"Unexpected error during hardware initialization: {e}")
#     raise

# --- Core Logic Components ---

logger.info("ROV Initialized Successfully.")


def arm() -> bool:
    """Arms the ROV, enabling motor control."""
    global is_armed
    if not communication.is_connected():
        logger.warning("Cannot arm: ROV not connected.")
        return False
    is_armed = True
    logger.info("ROV Armed.")
    return True


def disarm() -> bool:
    """Disarms the ROV, disabling motor control and stopping motors."""
    global is_armed
    is_armed = False
    motors.stop_all_motors()
    logger.info("ROV Disarmed. Motors stopped.")
    return True


def set_movement_targets(x: float, y: float, z: float, yaw: float):
    """
    Sets the desired movement targets for the ROV.
    Args:
        x (float): Forward/backward thrust (-1.0 to 1.0).
        y (float): Strafe left/right thrust (-1.0 to 1.0).
        z (float): Up/down thrust (-1.0 to 1.0).
        yaw (float): Rotational thrust (-1.0 to 1.0 for turn rate or torque).
    """
    global current_target_movement
    current_target_movement = {"x": x, "y": y, "z": z, "yaw": yaw}
    logger.debug(
        f"Movement targets set: {current_target_movement}")

    if is_armed and communication.is_connected():
        thruster_outputs = control.calculate_thruster_outputs(
            x, y, z, yaw
        )
        current_thruster_outputs = thruster_outputs
        # Assumes MotorController has this
        motors.set_thruster_speeds(thruster_outputs)
    elif not is_armed:
        motors.stop_all_motors()  # Ensure motors are stopped if disarmed
    else:
        logger.warning("Cannot apply movement: ROV not connected.")


def update_telemetry(force_update=False):
    """
    Updates telemetry data from sensors.
    Args:
        force_update (bool): If True, updates regardless of time since last update.
    """
    '''global telemetry_data, last_telemetry_update, last_telemetry_warning
    if not communication.is_connected():
        if time.monotonic() - last_telemetry_warning > LOG_REPEAT_INTERVAL:
            logger.warning("Cannot update telemetry: ROV not connected.")
            last_telemetry_warning = time.monotonic()
        # Potentially clear or mark telemetry as stale
        telemetry_data = {k: None for k in telemetry_data}
        telemetry_data["camera_pan"] = telemetry_data.get(
            "camera_pan", 0)  # Keep last known cam angles
        telemetry_data[ "camera_tilt"] = telemetry_data.get(
            "camera_tilt", 0)
        return'''

    # Limit update frequency unless forced
    now = time.time()
    # Update at most ~20Hz
    if not force_update and (now - last_telemetry_update < 0.05):
        return

    logger.debug("Updating telemetry...")
    updated_data = telemetry.collect_all_data()
    telemetry_data.update(updated_data)  # Merge new data
    last_telemetry_update = now
    logger.debug(f"Telemetry updated: {telemetry_data}")


def get_current_state() -> dict:
    """
    Returns a comprehensive dictionary of the robot's current state.
    """
    return {
        "is_armed": is_armed,
        "is_connected": communication.is_connected() if communication else False,
        "target_movement": current_target_movement,
        "thruster_outputs": current_thruster_outputs,
        "telemetry": telemetry_data,
        "timestamp": time.time()
    }


def set_camera_pan_tilt(pan_angle: int | None = None, tilt_angle: int | None = None) -> bool:
    """Controls the camera pan and/or tilt via TTY commands."""
    success = True
    if not communication.is_connected():
        logger.warning("Cannot control camera: ROV not connected.")
        return False

    if pan_angle is not None:
        if servo.camera_pan(pan_angle):
            telemetry_data["camera_pan"] = pan_angle
        else:
            logger.warning(f"Failed to set camera pan to {pan_angle}")
            success = False

    if tilt_angle is not None:
        if servo.camera_tilt(tilt_angle):
            telemetry_data["camera_tilt"] = tilt_angle
        else:
            logger.warning(
                f"Failed to set camera tilt to {tilt_angle}")
            success = False
    return success


def get_camera_feed_url() -> str | None:
    """
    Placeholder for getting a camera feed URL.
    Actual video streaming typically doesn't go through the main TTY.
    This might return an RTSP URL or similar if the camera has an IP interface.
    """
    # If your camera is an IP camera, you might store its URL here or in config.
    # For a USB camera with OpenCV, the GUI would handle the frame capture directly.
    logger.info(
        "get_camera_feed_url() called. Note: TTY interface is for control, not primary video.")
    return "http://<ROV_CAMERA_IP_ADDRESS_OR_STREAM_ENDPOINT>"  # Example


def shutdown():
    """Safely shuts down the ROV."""
    logger.info("ROV Shutting down...")
    disarm()  # This also stops motors
    if hasattr(communication,'communication') and communication.is_connected():
        communication.disconnect()
    # if hasattr('camera_video') and camera_video.isOpened():
    #     camera_video.release()
    logger.info("ROV Shutdown complete.")


def start():
    
    global roll, pitch, yaw,direction
    global latest_joystick, mode, button
    global esp_depth,esp_battery ,change_pitch_depth
    global error_atuo
    global roll1, pitch1, yaw1
    imusensor = None
    prev_joystick = None
    pid_yaw = pid.PIDController(Kp=0.1, Ki=0.005, Kd=0.2)
    pid_depth = pid.PIDController(Kp=0.8, Ki=0.000, Kd=0.0)
    #pid_pitch = pid.PIDController(Kp=1.0, Ki=0.0, Kd=0.00)
    roll1, pitch1, yaw1 = 0,0,0
    zarib = 0.017  #3.141592 / 180.0
    send_corr_yaw1, send_corr_yaw2, send_corr_pitch1, send_corr_pitch2 = 0,0,0,0
    correction_direction = "0"
    try:
        try:
            imusensor = IMUSensor()
        except OSError as e:
            logger.error(f"IMU init failed: {e}")
            imusensor = None

        logger.info("Starting ROV...")
        communication.connect()


        while True:
            
            if imusensor is not None:
                error_atuo = 0.0
                try:
                    error_atuo = 0.0
                    roll1, pitch1, yaw1 = imusensor.get_orientation()
                    roll = roll1 * zarib
                    pitch = pitch1 * zarib
                    yaw = yaw1 * zarib
                except OSError as e:
                    error_atuo = 1.0
                    logger.error(f"IMU read failed: {e}")
                    roll1, pitch1, yaw1 = 0.0, 0.0, 0.0
                    mode = "MANUAL"

            else:
                error_atuo = 1.0
                roll1, pitch1, yaw1 = 0.0, 0.0, 0.0
            if mode == "MANUAL":
                if (latest_joystick != prev_joystick):          
                    if communication.send_command(latest_joystick):
                        logger.info(f"Sent latest_joystick to serial: {latest_joystick}")
                        prev_joystick = latest_joystick  # ????????? ???? ??????? ?? ?????
                    else:
                        logger.warning(f"Failed to send latest_joystick: {latest_joystick}")

            elif mode == "AUTO":
                
                try:
                   
                    error_atuo = 0.0
                    direction = latest_joystick[0]
                    direction_pitch = latest_joystick[7]
                    correction_direction = direction_pitch
                   
                    base_left = int(latest_joystick[1:3])
                    base_right = int(latest_joystick[5:7])

                    correction_yaw = pid_yaw.pid_yaw(direction, button, yaw1)
                    if direction in ["2", "8","5"]:                         
                        send_corr_yaw1 = max(0, min(99, base_left - correction_yaw))
                        send_corr_yaw2 = max(0, min(99, base_right + correction_yaw))
                    else:                        
                        send_corr_yaw1 = base_left
                        send_corr_yaw2 = base_right
                       

                    if button == '4':
                        change_pitch_depth = "depth"
                    elif button in ['6', '7', '2'] or direction_pitch in ['2', '8']:
                        change_pitch_depth = "pitch"

                    if(change_pitch_depth == "pitch"):       
                        #correction_pitch = pid_pitch.pid_pitch(direction_pitch, button, pitch1)
                        #send_corr_pitch1 = max(0, min(99, int(latest_joystick[8:10]) - correction_pitch))
                        #send_corr_pitch2 = max(0, min(99, int(latest_joystick[10:12]) + correction_pitch))
                        correction_direction = direction_pitch
                        send_corr_pitch1 = int(latest_joystick[8:10])
                        send_corr_pitch2 = int(latest_joystick[10:12])
                    
                    elif(change_pitch_depth == "depth"):                
                        correction_power, correction_direction = pid_depth.pid_depth(button, esp_depth)
                        send_corr_pitch1 = correction_power
                        send_corr_pitch2 = correction_power
                        
                    #send_esp = f"{latest_joystick[0:7]}{correction_direction}{send_corr_pitch1:02d}{send_corr_pitch2:02d}{latest_joystick[12]}"
                    send_esp = f"{latest_joystick[0]}{send_corr_yaw1:02d}{latest_joystick[3:5]}{send_corr_yaw2:02d}{latest_joystick[7]}{send_corr_pitch1:02d}{send_corr_pitch2:02d}{latest_joystick[12]}"

                    if communication.send_command(send_esp):
                        logger.info(f"Sent latest_joystick to serial: {send_esp}")
                        prev_joystick = latest_joystick  
                    else:
                        logger.warning(f"Failed to send latest_joystick: {send_esp}")

                except OSError as e:
                    error_atuo = 1.0
                    if communication.send_command(latest_joystick):
                        logger.info(f" error not imu Sent latest_joystick to serial : {latest_joystick}")
                    else:
                        logger.warning(f"error not imuFailed to send latest_joystick: {latest_joystick}")

            time.sleep(0.05)
            response = communication.read_line()  
            if response is not None:
                #logger.info(f"Received from ESP: {response}") 
                #print(f"ESP Response: {response}")   
                
                for line in response.split('\n'):
                    line = line.strip()
                    if ':' in line:
                        try:
                            key, value = line.split(':', 1)
                            key = key.strip().lower() 
                            value = value.strip()
                            
                            if key == 'depth':
                                esp_depth = float(value)
                            elif key == 'battery':
                                esp_battery = int(value)
                        except ValueError as e:
                            logger.error(f"Error parsing value in line: {line} - Error: {e}")
                        
    except KeyboardInterrupt:
        pass
    finally:
        shutdown()

# Example usage (for testing this module in isolation)
if __name__ == "__main__":
    try:
        
        if communication.is_connected():
            print("ROV Connected. Arming...")
            arm()
            print(f"ROV Armed: {is_armed}")

            print("\nUpdating telemetry...")
            update_telemetry(force_update=True)
            print(f"Initial State: {get_current_state()}")

            print("\nSetting movement: Forward 0.5")
            set_movement_targets(x=0.5, y=0.0, z=0.0, yaw=0.0)
            # Allow time for command to be processed notionally
            time.sleep(0.1)
            print(f"State after move command: {get_current_state()}")

            print("\nSetting camera pan to 30, tilt to -10")
            set_camera_pan_tilt(pan_angle=30, tilt_angle=-10)
            print(f"State after camera command: {get_current_state()}")

            time.sleep(1)
            print("\nDisarming ROV...")
            disarm()
            print(f"ROV Armed: {is_armed}")

            shutdown()
        else:
            print("Could not run example: ROV not connected (this is expected if no virtual serial port is set up).")

    except ConnectionError as e:
        print(
            f"Connection Error: {e}. Ensure a serial port is available or mock hardware_interface.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
