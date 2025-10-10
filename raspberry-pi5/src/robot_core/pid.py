import time

class PIDController:
    def __init__(self, Kp, Ki, Kd, integral_limit=10.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None
        self.last_darage = None

        self.target_yaw = None
        self.last_darage_yaw = None
        self.current_IMU_yaw = None

        self.target_pitch = None
        self.last_darage_pitch = None
        self.current_IMU_pitch = None

        self.target_depth = None

        self.current_depth = None

    def smart_angle_error(self, target, current):  # for yaw
        error = target - current
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
        return error
    def linear_error(self, target, current): #  for depth
        return target - current
    
    def compute(self, setpoint, current_value, mode='angle'):
        if mode == 'angle':
            error = self.smart_angle_error(setpoint, current_value)
        else:
            error = self.linear_error(setpoint, current_value)
         
        current_time = time.time()
        
        if self.previous_time is None:
            dt = 0.05
            derivative = 0.0
        else:
            dt = current_time - self.previous_time
            if dt <= 0 or dt > 0.2:
                dt = 0.05
            
        derivative = (error - self.previous_error) / dt if self.previous_error is not None else 0.0
        output = (self.Kp * error) + (self.Kd * derivative)
        self.previous_error = error
        self.previous_time = current_time
        
        return output
    
    def pid_yaw(self, joystick_yaw, button, IMU_yaw):
        self.current_IMU_yaw = IMU_yaw
        darage = joystick_yaw
        darage_2 = button

        if darage != self.last_darage:
            self.last_darage = darage
            self.integral = 0.0
            self.previous_error = None
            self.previous_time = None
            
            if darage == '8':  
                self.target_yaw = IMU_yaw 
            elif darage == '2':   
                self.target_yaw = IMU_yaw  
            elif darage == '5': 
                self.target_yaw = IMU_yaw


        if darage_2 == '8':
            self.target_yaw = (self.target_yaw - 5) % 360 
        elif darage_2 == '9':
            self.target_yaw = (self.target_yaw + 5) % 360 

        correction = self.compute(self.target_yaw, self.current_IMU_yaw)
        correction = max(-50, min(50, correction))
        
        print(f"------------------target_yaw: {self.target_yaw}, current: {self.current_IMU_yaw}, correction: {correction}------------------")
        return int(correction)

  

    def pid_pitch(self, joystick_pitch, button, IMU_pitch):
        self.current_IMU_pitch = IMU_pitch
        darage_2 = button

        if self.target_pitch is None:
            self.target_pitch = IMU_pitch

        if darage_2 != self.last_darage_pitch:
            self.last_darage_pitch = darage_2
            self.integral = 0.0
            self.previous_error = None
            self.previous_time = None


            if darage_2 == '6': 
                self.target_pitch += 10
            elif darage_2 == '7':  
                self.target_pitch -= 10
            self.target_pitch = max(min(self.target_pitch, 90), -90)

        correction = self.compute(self.target_pitch, self.current_IMU_pitch)
        return int(correction)
    

    def pid_depth(self, button, current_depth):
        if current_depth < 0:
            self.current_depth = 0.0
        else:
            self.current_depth = current_depth
        
            
        if button == '4':
            self.target_depth = self.current_depth
            self.integral = 0.0
            self.previous_error = None
            self.previous_time = None

        elif button == '2':
            self.target_depth = None
            return 0, "5"

        if self.target_depth is None:
             return 0, "5"
        
        correction = self.compute(self.target_depth, self.current_depth, mode='linear')
        

        if correction > 1: 
            return min(int(abs(correction)), 99), "2" 
        elif correction < -1:
            return min(int(abs(correction)), 99), "8" 
        else:
            return 0, "5"  
        


if __name__ == '__main__':
    depth = PIDController(1.8,0.00, 0.001)
    
    print("Setting target depth with button '4'")
    print(f"return corection: {depth.pid_depth('4', 5)}") 
    print(f"max in function:{max(150, 99)}")
    print("\nTesting depth changes:")
    for i in range(0, 40):
        correction, direction = depth.pid_depth(None, i) 
        print(f"Current depth: {i}, Correction: {correction}, Direction: {direction}")
        time.sleep(0.2)
