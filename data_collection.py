#!/usr/bin/env python3

#This script is tailored to collect the sensor data for various stages of SMSS Active Smart-Stick.  
import smbus
import pyfirmata
import time
import csv
import RPi.GPIO as GPIO
from datetime import datetime

label = 'Test'

now = datetime.now()
curr_date_time = now.strftime("%d-%b-%Y %I:%M:%S %p")

board = pyfirmata.Arduino('/dev/ttyACM0')
print("Firmata Communication Successfully started")

en = 22
pwm = 15
ina = 16
inb = 18
logpin = 13 
GPIO.setmode(GPIO.BOARD)
GPIO.setup(logpin, GPIO.IN)
GPIO.setup(en, GPIO.OUT)
GPIO.setup(pwm, GPIO.OUT)
GPIO.setup(ina, GPIO.OUT)
GPIO.setup(inb, GPIO.OUT)



    #some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47


def MPU_Init():
        #write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
        #Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
        #Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)
        #Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
        #Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
        #Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)

        #concatenate higher and lower value
        value = ((high << 8) | low)

        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value


bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
time.sleep(1)
Device_Address = 0x68   # MPU6050 device address

MPU_Init()
print ("IMU  initiated - Reading Data of Gyroscope and Accelerometer")

it = pyfirmata.util.Iterator(board)
it.start()

red = 8
green = 9
blue = 10


forceA = board.analog[0]
forceB = board.analog[1]
forceC = board.analog[2]
forceD = board.analog[3]

forceA.enable_reporting()
forceB.enable_reporting()
forceC.enable_reporting()
forceD.enable_reporting()
print("Force sensors activated")
    
storagepath = '/home/pi/Jyothish/data_'+ label + '.csv'
print(storagepath)

with open(storagepath, 'a', encoding= 'UTF8') as f:
    writer = csv.writer(f)
    print("Data logging initiated")

    # i = ['xxxxxxxxx-', curr_date_time, '-xxxxxxxxxx']
    # writer.writerow(i)
    print(curr_date_time)
    
    heading = ['DateTime','Acc-X','Acc-Y','Acc-Z','Gyro-X','Gyro-Y','Gyro-Z','Force-A','Force-B','Force-C','Force-D', 'Label']
    writer.writerow(heading)
    
    # board.digital[en].write(1)
    # board.digital[pwm].write(1)
    # board.digital[ina].write(1)
    # board.digital[inb].write(0)
    
    GPIO.output(en, GPIO.HIGH)
    GPIO.output(pwm, GPIO.HIGH)
    GPIO.output(ina, GPIO.HIGH)
    GPIO.output(inb, GPIO.LOW)


    j=0
    while True:
        # board.digital[blue].write(1)
        time.sleep(0.1)
        #Read Accelerometer raw value
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)
        #Read Gyroscope raw value
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)
        #Read Force Sensor values
        fA = forceA.read()
        fB = forceB.read()
        fC = forceC.read()
        fD = forceD.read()
        #Full scale range +/- 250 degree/C as per sensitivity scale factor
        Ax = acc_x/16384.0
        Ay = acc_y/16384.0
        Az = acc_z/16384.0
        Gx = gyro_x/131.0
        Gy = gyro_y/131.0
        Gz = gyro_z/131.0

        row = [curr_date_time, round(Ax,2), round(Ay,2), round(Az,2), round(Gx,2), round(Gy,2), round(Gz,2), fA, fB, fC, fD, label]
        
        if (GPIO.input(logpin)) == 1:
            writer.writerow(row)
            print (row)
            # board.digital[blue].write(0)
            # board.digital[green].write(1)
            board.digital[red].write(0)
            board.digital[green].write(1)
            j=0     
        #print (u'\u000a', "Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", u'\u000a',"Ax=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az, u'\u000a',"ForceA = ", fA, "ForceB = ", fB, "ForceC = ", fC, "ForceD = ", fD) 	
        else:
            # board.digital[green].write(0)
            if j == 0:
                nextset = ['-------------------------------------']
                writer.writerow(nextset)
                print(nextset)
                board.digital[green].write(0)
                board.digital[red].write(1)
                j= j + 1       
        f.flush()
        # board.digital[blue].write(0)
        
       
