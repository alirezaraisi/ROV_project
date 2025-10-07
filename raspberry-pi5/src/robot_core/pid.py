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

    def smart_angle_error(self, target, current):
        error = target - current
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
        return error
    def linear_error(self, target, current):
        """محاسبه خطای خطی ساده برای عمق"""
        return target - current
    '''    
    def compute(self, setpoint, current_value, mode ='angle'):
        """تابع محاسبه PID با پشتیبانی از دو حالت زاویه و خطی"""
        if mode == 'angle':
            error = self.smart_angle_error(setpoint, current_value)
        else:  # linear mode
            error = self.linear_error(setpoint, current_value)
        current_time = time.time()

        if self.previous_time is None:
            dt = 0.001
            derivative = 0.0
        else:
            dt = current_time - self.previous_time
            if dt <= 0:
                dt = 0.001
            derivative = (error - self.previous_error) / dt

        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)

        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        self.previous_error = error
        self.previous_time = current_time
        return output
    '''
    def compute(self, setpoint, current_value, mode='angle'):
        if mode == 'angle':
            error = self.smart_angle_error(setpoint, current_value)
        else:
            error = self.linear_error(setpoint, current_value)
        
        # Dead Zone روی خطا
        dead_zone_error = 0.03  # 10cm
        if abs(error) < dead_zone_error:
            error = 0.0
        
        current_time = time.time()
        
        if self.previous_time is None:
            dt = 0.05
            derivative = 0.0
        else:
            dt = current_time - self.previous_time
            if dt <= 0 or dt > 0.2:
                dt = 0.05
            
            raw_derivative = (error - self.previous_error) / dt if self.previous_error is not None else 0.0
            
            # فیلتر قوی روی مشتق
            alpha = 0.3
            if hasattr(self, 'filtered_derivative'):
                derivative = alpha * raw_derivative + (1 - alpha) * self.filtered_derivative
            else:
                derivative = raw_derivative
            self.filtered_derivative = derivative

        output = (self.Kp * error) + (self.Kd * derivative)
        
        # Dead Zone اضافی روی خروجی
        dead_zone_output = 3.0
        if abs(output) < dead_zone_output:
            output = 0.0
        
        self.previous_error = error
        self.previous_time = current_time
        
        return output
    
    def pid_yaw(self, joystick_yaw, button, IMU_yaw):
        self.current_IMU_yaw = IMU_yaw
        darage = joystick_yaw
        darage_2 = button

        # ریست PID هنگام تغییر جهت اصلی
        if darage != self.last_darage:
            self.last_darage = darage
            self.integral = 0.0
            self.previous_error = None
            self.previous_time = None
            
            # منطق جدید برای تنظیم target_yaw
            if darage == '8':  # حرکت به جلو - حفظ جهت فعلی
                self.target_yaw = IMU_yaw  # حفظ جهت فعلی
            elif darage == '2':  # حرکت به عقب - حفظ جهت فعلی  
                self.target_yaw = IMU_yaw  # حفظ جهت فعلی
            elif darage == '5':  # توقف - حفظ جهت
                self.target_yaw = IMU_yaw

        # کنترل دستی با دکمه‌های ۸ و ۹
        if darage_2 == '8':
            self.target_yaw = (self.target_yaw - 5) % 360  # چرخش آرام به چپ
        elif darage_2 == '9':
            self.target_yaw = (self.target_yaw + 5) % 360  # چرخش آرام به راست

        # محاسبه تصحیح PID
        correction = self.compute(self.target_yaw, self.current_IMU_yaw)
        
        # محدود کردن خروجی
        correction = max(-50, min(50, correction))  # محدودیت برای جلوگیری از نوسان شدید
        
        print(f"------------------target_yaw: {self.target_yaw}, current: {self.current_IMU_yaw}, correction: {correction}------------------")
        return int(correction)

  

    def pid_pitch(self, joystick_pitch, button, IMU_pitch):
        self.current_IMU_pitch = IMU_pitch
        darage_2 = button

        # ???????? ????? ??? target_pitch ???? ?????
        if self.target_pitch is None:
            self.target_pitch = IMU_pitch

        # ??? ?? ???? ????? ????? ??? ?? ????? ???
        if darage_2 != self.last_darage_pitch:
            self.last_darage_pitch = darage_2
            self.integral = 0.0
            self.previous_error = None
            self.previous_time = None

            # ????? ??? ?? ???? ????
            if darage_2 == '6':  # ?????? 10 ????
                self.target_pitch += 10
            elif darage_2 == '7':  # ???? 10 ????
                self.target_pitch -= 10

            # ????? ???? pitch ?? ?????? ????? (????? ??? -90 ?? 90)
            self.target_pitch = max(min(self.target_pitch, 90), -90)

        # ?????? ????? PID
        correction = self.compute(self.target_pitch, self.current_IMU_pitch)
        return int(correction)
    
    def pid_depth(self, button, current_depth):
        # تصحیح عمق منفی
        if current_depth < 0:
            self.current_depth = 0.0
        else:
            self.current_depth = current_depth
        
            
        if button == '4':
            self.target_depth = self.current_depth
            # ریست پارامترهای PID برای شروع جدید
            self.integral = 0.0
            self.previous_error = None
            self.previous_time = None
        # حالت 2: دکمه 1 - غیرفعال کردن کنترل PID (عمق آزاد)
        elif button == '2':
            self.target_depth = None
            return 0, "5"
        
        # اگر تارگت عمق تنظیم نشده باشد (حالت عادی یا اولیه)
        if self.target_depth is None:
             return 0, "5"
        

        # استفاده از تابع compute موجود برای محاسبه PID
        correction = self.compute(self.target_depth, self.current_depth, mode='linear')
        
        # منطق ساده: فقط جهت را بر اساس علامت تعیین کن
        if correction > 1:  # آستانه کوچک برای جلوگیری از نوسان
            return min(int(abs(correction)), 99), "2"  # پایین
        elif correction < -1:
            return min(int(abs(correction)), 99), "8"  # بالا
        else:
            return 0, "5"  # ثابت ماندن
        
        #return int(correction)



'''
    def pid_yaw(self, joystick_yaw,button, IMU_yaw, sabt=0, flag_yaw=0):
        self.current_IMU_yaw = IMU_yaw
        darage = joystick_yaw
        darage_2 = button

        if darage != self.last_darage:
            self.last_darage = darage
            # --- ???: ??????? ?? ??? ???? ? ??????? ????? ????? ??? ---
            self.integral = 0.0
            self.previous_error = None
            self.previous_time = None
            # ??? ?? ??? ????? ????? ??? ????? ?? (???? ?? ????? ????)
            if darage == '8':
                self.target_yaw = IMU_yaw + 0
            elif darage == '2':
                self.target_yaw = IMU_yaw + 0
            else:
                self.target_yaw = IMU_yaw

            # ?????????? ??? ?? ?????? 0..360 (???? ??????? ? ???? ????)
            self.target_yaw = self.target_yaw % 360



        # --- ????? ????? ?? darage? ??????? ???? ?? ???? 7 ? 8 ---
        if darage_2 == '8':
            self.target_yaw = (self.target_yaw - 10) % 360
        elif darage_2 == '9':
            self.target_yaw = (self.target_yaw + 10) % 360

        # ?????? ????? PID ???? ?? ??? ????????
        print(f"------------------zavie_hadaf : {self.target_yaw}------------------")
        correction = self.compute(self.target_yaw, self.current_IMU_yaw)
        return int(correction)

'''

'''
def pid_yaw(self, joystick_yaw, button, IMU_yaw):
    self.current_IMU_yaw = IMU_yaw
    darage = joystick_yaw
    darage_2 = button

    if darage != self.last_darage:
        self.last_darage = darage
        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None
        
        # تنظیم target_yaw بر اساس جهت joystick
        if darage == '8':  # جلو
            self.target_yaw = IMU_yaw  # حفظ جهت فعلی
        elif darage == '9':  # جلو-راست
            self.target_yaw = (IMU_yaw - 45) % 360
        elif darage == '6':  # راست (چرخش خالص)
            self.target_yaw = (IMU_yaw - 90) % 360  # چرخش 90 درجه به راست
        elif darage == '3':  # عقب-راست
            self.target_yaw = (IMU_yaw - 135) % 360
        elif darage == '2':  # عقب
            self.target_yaw = (IMU_yaw - 180) % 360
        elif darage == '1':  # عقب-چپ
            self.target_yaw = (IMU_yaw + 135) % 360
        elif darage == '4':  # چپ (چرخش خالص)
            self.target_yaw = (IMU_yaw + 90) % 360  # چرخش 90 درجه به چپ
        elif darage == '7':  # جلو-چپ
            self.target_yaw = (IMU_yaw + 45) % 360
        else:  # توقف
            self.target_yaw = IMU_yaw

    # تنظیم دقیق با دکمه‌ها
    if darage_2 == '8':
        self.target_yaw = (self.target_yaw - 5) % 360  # تنظیم دقیق به راست
    elif darage_2 == '9':
        self.target_yaw = (self.target_yaw + 5) % 360  # تنظیم دقیق به چپ

    # محاسبه تصحیح PID
    correction = self.compute(self.target_yaw, self.current_IMU_yaw)
    return int(correction)
'''
        


        
    

