import re
import time
#from robot_core import log
#from hardware_interface import communication
#logger = log.getLogger(__name__)

class GamepadController:
    # Constants
    GEAR_DELAY = 0.3
    PITCH_INCREASE_DELAY = 0.1
    PITCH_INCREASE_STEP = 0.02
    

    def __init__(self):
        # Instance variables
        self.gear_move = 0.25
        self.last_gear_change = 0
        self.flag_change = 0
        self.gear_depth = 0.25
        self.last_gear_change1 = 0
        self.flag_change1 = 0
        self.pitch_power = 0.0
        self.last_pitch_increase = 0
        self.gamepad = None
        
        # ???????? ???? ???? ????? ?????? ????
        self.prev_gear_move = 0.1
        self.prev_gear_depth = 0.1
        self.prev_movement = 500
        self.prev_depth = 500
        self.prev_mode = 0
        self.prev_button = 5


        self.prev_raw_data = None

        self.update_interval = 0.05  # 50ms (20 ??? ?? ?????)
        self.last_update_time = 0



    def mmap(self, value, leftMin, leftMax, rightMin, rightMax):
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin
        if leftSpan == 0:  # ??????? ?? ????? ?? ???
            return int(rightMin)
        valueScaled = float(value - leftMin) / float(leftSpan)
        return int(rightMin + (valueScaled * rightSpan))
 
    def get_movement(self, x, y, dead_zone=5, straight_angle_margin=80):
        center = 128  # ??? ?????????

        dx = x - center
        dy = y - center

        if abs(dx) < dead_zone and abs(dy) < dead_zone:
            # Reset gear to 0.25 when stopping
            self.gear_move = 0.25
            return "5000000"

        direction = "5"
        s = 0

        # ??? ????? ????? ????? ????? ??? ?? ????
        if abs(dy) > abs(dx) + straight_angle_margin:  
            # ???? ????? (? ?? ?)
            if dy < -dead_zone:
                direction = "8"  # ????
                s = self.mmap(abs(dy), dead_zone, center, 0, 99)
            elif dy > dead_zone:
                direction = "2"  # ?????
                s = self.mmap(abs(dy), dead_zone, center, 0, 99)

        elif abs(dx) > abs(dy) + straight_angle_margin:
            # ???? ???? (? ?? ?)
            if dx < -dead_zone:
                direction = "4"  # ??
                s = self.mmap(abs(dx), dead_zone, center, 0, 99)
            elif dx > dead_zone:
                direction = "6"  # ????
                s = self.mmap(abs(dx), dead_zone, center, 0, 99)

        else:
            # ???????
            if dx > 0 and dy < 0:
                direction = "9"  # ????-????
            elif dx < 0 and dy < 0:
                direction = "7"  # ????-??
            elif dx > 0 and dy > 0:
                direction = "3"  # ?????-????
            elif dx < 0 and dy > 0:
                direction = "1"  # ?????-??

            s = self.mmap(max(abs(dx), abs(dy)), dead_zone, center, 0, 99)

        s = int(s * self.gear_move)
        return f"{direction}{s:02d}{s:02d}{s:02d}"

    
    def get_button(self, buttons):
        direction = "0"
        if buttons == 2:   # daire, binary: & (1 << 11):   vasat joystick rast
            direction = "1"   #
        elif buttons == 4:   # moraba, binary: & (1 << 10):   vasat joystick chap
            direction = "2"
        elif buttons ==  8: # mosalas, binary: & (1 << 3):
            direction = "3"
        elif buttons == 16:  # zarbedar, binary: & (1 << 4):
            direction = "4"
        elif buttons == 4096: # felesh bala, binary: & (1 << 12): 
            direction = "6"
        elif buttons == 8192:  # felesh paien, binary: & (1 << 13):
            direction = "7"
        elif buttons == 16384 :  # felesh chap ,binary: & (1 << 14)
            direction = "8"
        elif buttons == 32768:  # felesh rast, binary: & (1 << 15):
            direction = "9"
        return f"{direction}"
    
    def light(self, buttons):
        status = "0"
        if buttons == 2048:   # daire, binary: & (1 << 11):   vasat joystick rast
            status = "1"   #on
        elif buttons == 1024:   # moraba, binary: & (1 << 10):   vasat joystick chap
            status = "2"    #off
        return status     
   
   
   
    def get_mode(self, buttons,buttons2):
        mode = "0"
        if buttons==193 :  # start AUTO
            mode = "1"
        elif buttons2 == 2 :  # select MANUAL
            mode = "2"
        return f"{mode}"

    def get_gear_move(self, up_down_gear, up_down_gear_analog):
        # Round and clamp gear position between 0.0-1.0
        self.gear_move = round(self.gear_move, 2)
        self.gear_move = max(0.25, min(self.gear_move, 1.0))
        
        # Check gear change cooldown
        current_time = time.time()
        if current_time - self.last_gear_change < self.GEAR_DELAY:
            return self.gear_move
        
         # Shift up (1) or down (4) by 0.2 increments
        if up_down_gear == 32 and self.gear_move < 1.0:
            self.gear_move += 0.25
            self.last_gear_change = current_time
        elif up_down_gear_analog >= 220 and self.gear_move > 0.25:
            self.gear_move -= 0.25
            self.last_gear_change = current_time
        
        # Ensure final value is rounded and clamped    
        self.gear_move = round(self.gear_move, 2)
        self.gear_move = max(0.25, min(self.gear_move, 1.0))
        return self.gear_move
    
    def get_gear_depth(self, up_down_gear,up_down_gear_analog):
        self.gear_depth = round(self.gear_depth, 2)
        self.gear_depth = max(0.25, min(self.gear_depth, 1.0))
        current_time = time.time()
        if current_time - self.last_gear_change1 < self.GEAR_DELAY:
            return self.gear_depth
        if up_down_gear == 1 and self.gear_depth < 1.0:
            self.gear_depth += 0.25
            self.last_gear_change1 = current_time
        elif up_down_gear_analog >= 220 and self.gear_depth > 0.25:
            self.gear_depth -= 0.25
            self.last_gear_change1 = current_time
        self.gear_depth = round(self.gear_depth, 2)
        self.gear_depth = max(0.25, min(self.gear_depth, 1.0))
        return self.gear_depth
    
    


    
    def get_depth(self, x, y, dead_zone=5, straight_angle_margin=60):
        center = 128  # ??? ?????????

        dx = x - center
        dy = y - center

        if abs(dx) < dead_zone and abs(dy) < dead_zone:
            #self.gear_depth = 0.25
            return "50000"

        direction = "5"
        s = 0

        # ??? ????? ????? ????? ????? ??? ?? ????
        if abs(dy) > abs(dx) + straight_angle_margin:  
            # ???? ????? (? ?? ?)
            if dy < -dead_zone:
                direction = "8"  # ????
                s = self.mmap(abs(dy), dead_zone, center, 0, 99)
            elif dy > dead_zone:
                direction = "2"  # ?????
                s = self.mmap(abs(dy), dead_zone, center, 0, 99)

        elif abs(dx) > abs(dy) + straight_angle_margin:
            # ???? ???? (? ?? ?)
            if dx < -dead_zone:
                direction = "4"  # ??
                s = self.mmap(abs(dx), dead_zone, center, 0, 99)
            elif dx > dead_zone:
                direction = "6"  # ????
                s = self.mmap(abs(dx), dead_zone, center, 0, 99)

        else:
            return "50000"

        s = int(s * self.gear_depth)
        return f"{direction}{s:02d}{s:02d}"
    

    
    


    def combine_data(self, msg_x, msg_y, msg_r, msg_z, msg_s, msg_t, msg_buttons, msg_buttons2):
        self.get_gear_move(msg_buttons,msg_s)
        self.get_gear_depth(msg_buttons2,msg_t)

        #mode = self.get_mode(msg_buttons,msg_buttons2)      
        movement = self.get_movement(msg_x, msg_y)
        depth = self.get_depth(msg_r,msg_z)
        buttons = self.get_button(msg_buttons)
        ligth = self.light(msg_buttons)

        send_esp = f"{movement}{depth}{ligth}"
        send_robot = f"{buttons}"

        return send_esp, send_robot


    
 
