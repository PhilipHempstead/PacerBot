# PacerBot
#This code is strictly for working on and improving the LiDAR sweep cycle algorithm. That is why the "mode2_steering()" function, which is intended to perform #computer vision, is empty. All of the main motor commands are commented out for this same reason as well. The main motor commands can be identified as commands #starting with "odrv0.axis0".

#If you scroll past all of these functions you will arrive at where the program starts for "mode 2". 


from odrive.enums import *
import math
import serial
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
from gpiozero import Servo

import concurrent.futures
import time
import keyboard

ser = serial.Serial("/dev/ttyAMA0", 115200)

servo_steer = Servo("BOARD32")
servo_steer.value = 0.05
servo_lidar = Servo("BOARD31")
servo_lidar.value = 0.05

def start_mode_2( pos_Kp, pos_Kd, ramp_rate ):

    target_pos = 8  #The target position is 8 ft infront of the runner
    Base_speed = 60  #May change this value later.
    #odrv0.axis0.controller.config.vel_ramp_rate = ramp_rate
	  #odrv0.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
	  #odrv0.axis0.requested_state = AXIS_STATE_SENSORLESS_CONTROL

    direction = 1
    last_error = 0
    while True:
        distance = rotate_lidar(direction) #"distance" is set equal to the closest value from the sweep.
        put = "c: " + str(distance) 
        print(put)
        print(" ")
        
        
        if direction == 1: #switch the "direction" variable so that the sweep cycle switches direction
            direction = 0
        else:
            direction = 1

        if distance != 100:
            pos_error = distance - target_pos #set "pos_error" equal to the difference between the target and actual position
            #calculate motor_adjustment with PID equation
            motor_adjustment = (pos_Kp*pos_error) #+ (pos_Kd*(pos_error - last_error))
            motor_adjustment = motor_adjustment * (-1)
            #print(motor_adjustment)
            if motor_adjustment > 40: #max out so robot can't exceed a speed of 100. I will remove this later
                motor_adjustment = 40
            if motor_adjustment < -60: #minimum speed adjustment of -60 so robot can't go backwards
                motor_adjustment = -60
            #odrv0.axis0.controller.input_vel = Base_speed + motor_adjustment #adjust the input_vel
            last_error = pos_error #re-set last error for the next cycle of the PID equation
        if keyboard.is_pressed('q'):
            return

#rotate_lidar() sets the positions of the LiDAR and then returns the shortest distance for each sweep. The servo's can be set to a value between 1 and -1.
#I currently have the rotation range between -0.06 and 0.06. I will adjust these values to get the correct angles.
def rotate_lidar(direction):

    if direction == 1: #if direction == 1, the lidar sweeps right
        servo_lidar.value = -0.06
        closest = getTFminiData()
        out = "0: " + str(closest) + "  pos: " + str(-0.06)
        print(out)
        for x in range(1,4):  #adjust the servo position and perform 3 more readings
            pos = -0.06 + (x*0.04)
            servo_lidar.value = servo_lidar.value + 0.04
            time.sleep(0.05)
            dist = getTFminiData()
            out = str(x) + ": " + str(dist) + "  pos: " + str(pos)
            print(out)

            if dist < closest:
                closest = dist
        return closest
    else: #if direction == 0, lidar sweeps left
        servo_lidar.value = 0.06
        closest = getTFminiData()
        out = "0: " + str(closest) + "  pos: " + str(0.06)
        print(out)
        for x in range(1,4):
            pos = 0.06 - (x*0.04)
            servo_lidar.value = servo_lidar.value - 0.06
            time.sleep(0.05)
            dist = getTFminiData()
            out = str(x) + ": " + str(dist) + "  pos: " + str(pos)
            print(out)

            if dist < closest:
        	    closest = dist
        return closest
    return -1

def getTFminiData():
    #print("function called")
    #while True:
        #time.sleep(0.25)
        count = ser.in_waiting
        #print("made it past ser.in_waiting")
        #print(count)
        if count > 8:
            #print("count is greater than 8")
            recv = ser.read(9)   
            ser.reset_input_buffer() 
            # type(recv), 'str' in python2(recv[0] = 'Y'), 'bytes' in python3(recv[0] = 89)
            # type(recv[0]), 'str' in python2, 'int' in python3 
            
            if recv[0] == 0x59 and recv[1] == 0x59:     #python3
                distance = recv[2] + recv[3] * 256
                distance = distance / 30.48
                #strength = recv[4] + recv[5] * 256
                #print('(', distance, ',', strength, ')')
                
                #print("Distance: ", distance, " ft")
                ser.reset_input_buffer()

                return float(distance)
        return 100
    
def mode2_steering( Kp, Kd, input_vel ):
    print("arrived")
    center = 104
    error = 0
    last_error = 0
    x_last = 104
    y_last = 72
    #Kp = 1
    #Kd = 20

    while True:
        if keyboard.is_pressed('q'):
            #slow_to_stop( input_vel )

            break
    return


#Program starts here. Kp, Kd, pos_Kp, pos_Kd and ramp_rate are all declared as global variables.

#Kp and Kd are the coefficients for the computer vision steering PID algorithm
Kp = float(input("Enter steering Kp: "))
Kd = float(input("Enter steering Kd: "))

#pos_Kp and pos_Kd are the coefficients for the main motor speed adjustment PID algorithm	
pos_Kp = float(input("Enter lidar Kp (10): "))
pos_Kd = float(input("Enter lidar Kd (100): "))
ramp_rate = float(input("Enter ramp rate: ")) #ramp_rate is the rate at which the main motor accelerates/decelerates to its new velocity
	

print("Press 'q' to quit: ")
try:
    if ser.is_open == False: #make sure LiDAR sensor is ready to be read
        ser.open()
    with concurrent.futures.ThreadPoolExecutor() as executor: #Start multiprocessing. 
        start = time.perf_counter()
        f1 = executor.submit( mode2_steering, Kp, Kd, 10 ). #mode2_steering() algorithm is for the computer vision/linefollowing
        f2 = executor.submit( start_mode_2, pos_Kp, pos_Kd, ramp_rate) #start_mode_2 starts the main motor speed adjustment algorithm
except KeyboardInterrupt:   # Ctrl+C
        #odrv0.axis0.requested_state = AXIS_STATE_IDLE
        if ser != None:
            ser.close()

finish = time.perf_counter()

print(f'Finished in {round(finish-start, 2)} second(s)')
