from socket import *
from time import ctime
import binascii
import RPi.GPIO as GPIO
import time
import threading
import serial
import cv2
from multiprocessing import Process
import sys
import math
import torch
import numpy as np
import imutils

#configure arduino Serial
arduinoSerial = serial.Serial('/dev/ttyACM0',9600)
#######################################
#############Signal pin defination##############
#######################################
GPIO.setmode(GPIO.BCM)

########LED prot defination#################
LED0 = 10
LED1 = 9
LED2 = 25

########Motor drive port defination#################
ENA = 13	#//L298 Enalbe A
ENB = 20	#//L298 Enable B
IN1 = 19	#//Motor port 1
IN2 = 16	#//Motor port 2
IN3 = 21	#//Motor port 3
IN4 = 26	#//Motor port 4

#######################################
#########Pin type setup and initialization##########
#######################################
GPIO.setwarnings(False)

########Ultrasonic port defination#################
ECHO = 4	#Ultrasonic receiving pin  
TRIG = 17	#Ultrasonic sending pin

##########Ultrasonic module pin type set#########
GPIO.setup(TRIG,GPIO.OUT,initial=GPIO.LOW)#ultrasonic module transmitting end pin set trig
GPIO.setup(ECHO,GPIO.IN,pull_up_down=GPIO.PUD_UP)#ultrasonic module receiving end pin set echo

#########led initialized to 000##########
GPIO.setup(LED0,GPIO.OUT,initial=GPIO.HIGH)
GPIO.setup(LED1,GPIO.OUT,initial=GPIO.HIGH)
GPIO.setup(LED2,GPIO.OUT,initial=GPIO.HIGH)

#########motor initialized to LOW##########
GPIO.setup(ENA,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(ENB,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)
pa = GPIO.PWM(ENA,100)
pb = GPIO.PWM(ENB,100)
pa.start(0)
pb.start(0)

####################################################
##Function name：Get_Distance()
##Function performance ultrasonic ranging，return distance(unit is centimeter）
##Entrance parameter：none
##Exit parameter：none
####################################################
def	Get_Distance():
    #time.sleep(0.1)
    GPIO.output(TRIG,GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(TRIG,GPIO.LOW)
    while not GPIO.input(ECHO):
                pass
    t1 = time.time()
    while GPIO.input(ECHO):
                pass
    t2 = time.time()
    time.sleep(0.01)
    return math.floor((t2-t1)*34000/2)

def Motor_Forward():
    print('motor forward')
    GPIO.output(ENA,True)
    GPIO.output(ENB,True)
    GPIO.output(IN1,True)
    GPIO.output(IN2,False)
    GPIO.output(IN3,True)
    GPIO.output(IN4,False)
    GPIO.output(LED1,False)#Headlight's anode to 5V, cathode to IO port
    GPIO.output(LED2,False)#Headlight's anode to 5V, cathode to IO port

def Motor_Backward():
    print('motor_backward')
    GPIO.output(ENA,True)
    GPIO.output(ENB,True)
    GPIO.output(IN1,False)
    GPIO.output(IN2,True)
    GPIO.output(IN3,False)
    GPIO.output(IN4,True)
    GPIO.output(LED1,True)#Headlight's anode to 5V, cathode to IO port
    GPIO.output(LED2,False)#Headlight's anode to 5V, cathode to IO port

def Motor_TurnLeft():
    print('motor_turnleft')
    GPIO.output(ENA,True)
    GPIO.output(ENB,True)
    GPIO.output(IN1,True)
    GPIO.output(IN2,False)
    GPIO.output(IN3,False)
    GPIO.output(IN4,True)
    GPIO.output(LED1,False)#Headlight's anode to 5V, cathode to IO port
    GPIO.output(LED2,True)#Headlight's anode to 5V, cathode to IO port
def Motor_TurnRight():
    print('motor_turnright')
    GPIO.output(ENA,True)
    GPIO.output(ENB,True)
    GPIO.output(IN1,False)
    GPIO.output(IN2,True)
    GPIO.output(IN3,True)
    GPIO.output(IN4,False)
    GPIO.output(LED1,False)#Headlight's anode to 5V, cathode to IO port
    GPIO.output(LED2,True)#Headlight's anode to 5V, cathode to IO port
def Motor_Stop():
    print('motor_stop')
    GPIO.output(ENA,False)
    GPIO.output(ENB,False)
    GPIO.output(IN1,False)
    GPIO.output(IN2,False)
    GPIO.output(IN3,False)
    GPIO.output(IN4,False)
    GPIO.output(LED1,True)#Headlight's anode to 5V, cathode to IO port
    GPIO.output(LED2,True)#Headlight's anode to 5V, cathode to IO port

def Motor_RightSpeed(speed):
    #speed = translate(speed,-100,100,-100,100)
    pa.ChangeDutyCycle(abs(speed))
    if(speed<0):
        GPIO.output(IN1,False)
        GPIO.output(IN2,True)
        GPIO.output(ENA,True)
    elif(speed>0):
        GPIO.output(IN1,True)
        GPIO.output(IN2,False)
        GPIO.output(ENA,True)
    else:
        GPIO.output(IN1,False)
        GPIO.output(IN2,False)
        GPIO.output(ENA,False)
        
def Motor_LeftSpeed(speed):
    speed = 1.5*speed
    pb.ChangeDutyCycle(abs(speed))
    if(speed<0):
        GPIO.output(IN3,False)
        GPIO.output(IN4,True)
        GPIO.output(ENB,True)
    elif(speed>0):
        GPIO.output(IN3,True)
        GPIO.output(IN4,False)
        GPIO.output(ENB,True)
    else:
        GPIO.output(IN3,False)
        GPIO.output(IN4,False)
        GPIO.output(ENB,False)
def Move_Speed(speedleft, speedright):
    Motor_LeftSpeed(speedleft)
    Motor_RightSpeed(speedright)

def Move(x,speed):
    if(x<0):
        x=abs(x)
        x=translate(x,0,1,speed,-speed)
        Move_Speed(x,speed)
    else:
        x=translate(x,0,1,speed,-speed)
        Move_Speed(speed,x)

limits = {}
limits[1] = [0,180]#hand 1
limits[2] = [0,180]#hand 2
limits[3] = [0,180]#hand 3
limits[4] = [90,180]#hand 4
limits[7] = [0,150]#camera 1
limits[8] = [0,180]#camera 2

prev = {}
prev[1] = 180
prev[2] = 30
prev[3] = 90
prev[4] = 0
prev[7] = 90
prev[8] = 90

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

#Servo angle drive function
def SetServoAngle(servo, angle):
    arduinoSerial.write(str(str(servo)+":"+str(int(translate(max(min(angle,180),0),0,180,limits[servo][0],limits[servo][1])))+"\n").encode())
    GPIO.output(LED0,False)#Headlight's anode to 5V, cathode to IO port
    GPIO.output(LED1,False)#Headlight's anode to 5V, cathode to IO port
    GPIO.output(LED2,True)#Headlight's anode to 5V, cathode to IO port
    time.sleep(0.01)
    GPIO.output(LED0,True)#Headlight's anode to 5V, cathode to IO port
    GPIO.output(LED1,True)#Headlight's anode to 5V, cathode to IO port
    GPIO.output(LED2,True)#Headlight's anode to 5V, cathode to IO port

def servo_init():
    SetServoAngle(1,prev[1])
    SetServoAngle(2,prev[2])
    SetServoAngle(3,prev[3])
    SetServoAngle(4,prev[4])
    SetServoAngle(7,prev[7])
    SetServoAngle(8,prev[8])

def servo_input():
    time.sleep(2)
    servo_init()
    while True:
        print("Enter servo and angle")
        data = input()
        data = data.split(" ")
        servo = int(data[0])
        angle = int(data[1])
        SetServoAngle(servo, angle)

def camera_feed():
    time.sleep(2)
    servo_init()
    # define a video capture object
    vid = cv2.VideoCapture(0)
    while True:
        # Capture the video frame
        # by frame
        ret, frame = vid.read()
        # Display the resulting frame
        cv2.imshow('frame', frame)
          
        # the 'q' button is set as the
        # quitting button
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    # After the loop release the cap object
    vid.release()
    # Destroy all the windows
    cv2.destroyAllWindows()
    
def dist_func():
    while True:
        print("Distance: "+str(Get_Distance())+"cm")
def motor_func():
    time.sleep(3)
    servo_init()
    time.sleep(1)
    Move(0.6,100)
    time.sleep(1)
    Motor_Stop()
    
#make hand move to specific position
def hand_move(x):
    #hand parameters
    l1=4.5
    l2=9.5
    l3=14.0
    #compute limit
    limx = math.sqrt((l2+l3)**2 - l1**2)-1
    if(x > limx): return
    #compute the others
    l4=math.sqrt(l1**2 + x**2)
    a = math.degrees(math.atan(x/l1))
    b = math.degrees(math.acos((l2**2 + l4**2 -l3**2)/(2*l2*l4)))
    c = math.degrees(math.acos((l2**2 + l3**2 -l4**2)/(2*l2*l3)))
    ang1 = int(a+b-45)
    ang2 = int(c+10)
    SetServoAngle(2,ang2)
    SetServoAngle(1,ang1)
    
def hand_func():
    time.sleep(2)
    servo_init()
    while True:
        print("Enter distance")
        dist = input()
        hand_move(int(dist))
try:
    time.sleep(2)
    servo_init()
    while True:
        dist = Get_Distance()
        print(str(dist))
        hand_move(dist)
        time.sleep(1)
except KeyboardInterrupt:
    print("byeee!")


