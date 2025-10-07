
import os
import sys
import time
import smbus
from imusensor.MPU9250 import MPU9250
from imusensor.filters import madgwick



import time
from collections import deque


class IMUSensor:
    def __init__(self, calib_file_path="/home/pi/calib_real4.json", address=0x68, bus_num=1):
        self.bus = smbus.SMBus(bus_num)
        self.imu = MPU9250.MPU9250(self.bus, address)
        self.imu.begin()
        self.imu.loadCalibDataFromFile(calib_file_path)
        
        self.sensorfusion = madgwick.Madgwick(0.5)
        self.currTime = time.time()
        
        # سیستم جدید برای هموارسازی زمانی
        self.smooth_interval = 0.2  # 200 میلی‌ثانیه
        self.last_output_time = time.time()
        
        # بافر برای ذخیره‌سازی مقادیر اخیر
        self.roll_buffer = deque(maxlen=5)
        self.pitch_buffer = deque(maxlen=5) 
        self.yaw_buffer = deque(maxlen=5)
        
        # مقادیر فعلی هموار شده
        self.smooth_roll = 0.0
        self.smooth_pitch = 0.0
        self.smooth_yaw = 0.0
        
        # مقداردهی اولیه
        self.imu.readSensor()
        for i in range(10):
            newTime = time.time()
            dt = newTime - self.currTime
            self.currTime = newTime
            self.sensorfusion.updateRollPitchYaw(
                self.imu.AccelVals[0], self.imu.AccelVals[1], self.imu.AccelVals[2],
                self.imu.GyroVals[0], self.imu.GyroVals[1], self.imu.GyroVals[2],
                self.imu.MagVals[0], self.imu.MagVals[1], self.imu.MagVals[2], dt
            )
        
        # مقدار اولیه
        init_roll, init_pitch, init_yaw = self.sensorfusion.roll, self.sensorfusion.pitch, self.sensorfusion.yaw
        for _ in range(5):
            self.roll_buffer.append(init_roll)
            self.pitch_buffer.append(init_pitch)
            self.yaw_buffer.append(init_yaw)
        
        self.smooth_roll = init_roll
        self.smooth_pitch = init_pitch
        self.smooth_yaw = init_yaw
    
    def get_orientation(self):
        # همیشه سنسور را بخوان و فیلتر را آپدیت کن
        self.imu.readSensor()
        for i in range(5):
            newTime = time.time()
            dt = newTime - self.currTime
            self.currTime = newTime
            self.sensorfusion.updateRollPitchYaw(
                self.imu.AccelVals[0], self.imu.AccelVals[1], self.imu.AccelVals[2],
                self.imu.GyroVals[0], self.imu.GyroVals[1], self.imu.GyroVals[2],
                self.imu.MagVals[0], self.imu.MagVals[1], self.imu.MagVals[2], dt
            )
        
        # اضافه کردن به بافر
        current_roll, current_pitch, current_yaw = self.sensorfusion.roll, self.sensorfusion.pitch, self.sensorfusion.yaw
        self.roll_buffer.append(current_roll)
        self.pitch_buffer.append(current_pitch)
        self.yaw_buffer.append(current_yaw)
        
        # بررسی زمان برای خروجی جدید
        current_time = time.time()
        if current_time - self.last_output_time >= self.smooth_interval:
            self.last_output_time = current_time
            
            # محاسبه میانگین از بافر
            avg_roll = sum(self.roll_buffer) / len(self.roll_buffer)
            avg_pitch = sum(self.pitch_buffer) / len(self.pitch_buffer)
            avg_yaw = sum(self.yaw_buffer) / len(self.yaw_buffer)
            
            # هموارسازی نهایی با فیلتر کم‌گذر
            alpha = 0.3  # ضریب هموارسازی قوی
            self.smooth_roll = alpha * avg_roll + (1 - alpha) * self.smooth_roll
            self.smooth_pitch = alpha * avg_pitch + (1 - alpha) * self.smooth_pitch
            self.smooth_yaw = alpha * avg_yaw + (1 - alpha) * self.smooth_yaw
            
            # گرد کردن برای حذف نوسانات جزئی
            smooth_roll = round(self.smooth_roll)
            smooth_pitch = round(self.smooth_pitch)
            smooth_yaw = round(self.smooth_yaw)
            
            return smooth_roll, smooth_pitch, smooth_yaw
        else:
            # اگر زمان نرسیده، همان مقادیر قبلی را برگردان
            return self.smooth_roll, self.smooth_pitch, self.smooth_yaw


'''
class IMUSensor:
    def __init__(self, calib_file_path="/home/pi/calib_real4.json", address=0x68, bus_num=1):
        self.bus = smbus.SMBus(bus_num)
        self.imu = MPU9250.MPU9250(self.bus, address)
        self.imu.begin()
        self.imu.loadCalibDataFromFile(calib_file_path)
        
        self.sensorfusion = madgwick.Madgwick(0.8)
        self.currTime = time.time()
        
        # Initialize with 10 readings
        self.imu.readSensor()
        for i in range(20):
            newTime = time.time()
            dt = newTime - self.currTime
            self.currTime = newTime
            self.sensorfusion.updateRollPitchYaw(
                self.imu.AccelVals[0], self.imu.AccelVals[1], self.imu.AccelVals[2],
                self.imu.GyroVals[0], self.imu.GyroVals[1], self.imu.GyroVals[2],
                self.imu.MagVals[0], self.imu.MagVals[1], self.imu.MagVals[2], dt
            )        
    
    def get_orientation(self):
    
        self.imu.readSensor()
        for i in range(10):
            newTime = time.time()
            dt = newTime - self.currTime
            self.currTime = newTime
            self.sensorfusion.updateRollPitchYaw(
                self.imu.AccelVals[0], self.imu.AccelVals[1], self.imu.AccelVals[2],
                self.imu.GyroVals[0], self.imu.GyroVals[1], self.imu.GyroVals[2],
                self.imu.MagVals[0], self.imu.MagVals[1], self.imu.MagVals[2], dt
            )
        
        return self.sensorfusion.roll, self.sensorfusion.pitch, self.sensorfusion.yaw


'''

'''
imusensor = IMUSensor()
while True:
  
  roll1, pitch1, yaw1 = imusensor.get_orientation()
  print(f"roll:{roll1} , pitch:{pitch1}, yaw:{yaw1}")
  time.sleep(0.01)
'''