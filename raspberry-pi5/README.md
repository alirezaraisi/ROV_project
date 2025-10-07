# توضیحات استاندارد ارتباط با رابط کاربری
برای پیاده‌سازی استاندارد از توابع داخل فایل api استفاده می‌کنیم

# انواع پیام‌ها:

## ۱-پیام‌های ارسالی:(فرستادن به رابط کاربری)

برای ارسال پیام‌ها به رابط کاربری از توابع داخل message_builder.py استفاده میکنیم.

### یک نمونه از پیاده‌سازی:
```python

def send_raw_imu()->mavlink.MAVLink_raw_imu_message:
#we use the get functions written by the programmer here:
  compass_data=get_compass_data()
  gyro_data=get_gyro_data()
  accel_data=get_accel_data()



    return mavlink.MAVLink_raw_imu_message(
        time_usec=time_usec_handler(),
        xacc=accel_data["x"],
        yacc=accel_data["y"],
        zacc=accel_data["z"],
        xgyro=gyro_data["x"],
        ygyro=gyro_data["y"],
        zgyro=gyro_data["z"],
        xmag=compass_data["x"],
        ymag=compass_data["y"],
        zmag=compass_data["z"],
        id=65535,
        temperature=65535

    )





