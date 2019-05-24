import serial
import time

xy_arduino = serial.Serial('/dev/ttyUSB3', 9600)    # 2,3 motor
pm_arduino = serial.Serial('/dev/ttyUSB0', 9600)     # 4,5 mtor
first_arduino = serial.Serial('/dev/ttyUSB2',9600)  # 1 motor

delimiter = "/"

x_angle =-70   # 3 motor
y_angle = 41    # 2 motor

xy_angle = str(int(x_angle)) + delimiter + str(int(y_angle))
xy_angle = xy_angle.encode('utf-8')



z_angle = -70
temp_angle = -70
zt_angle = str(int(z_angle)) + delimiter + str(int(temp_angle))
zt_angle = zt_angle.encode('utf-8')


first_angle = 30
trash_angle = 30
first_angle_str = str(int(first_angle)) + delimiter + str(int(trash_angle))
first_angle_str = first_angle.encode('utf-8')

xy_arduino.write(zt_angle)
pm_arduino.write(xy_angle)
first_arduino.write(first_angle_str)

time.sleep(3)
