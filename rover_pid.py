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
limits[4] = [80,180]#hand 4
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
    
#make hand move to specific position
def hand_move(x):
    x-=1
    #hand parameters
    l1=4.5
    l1-=2
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
    SetServoAngle(1,ang1)
    SetServoAngle(2,ang2)
def catched():
    SetServoAngle(1,180)
    SetServoAngle(2,90)

time.sleep(3)
servo_init()
time.sleep(1)
# define a video capture object
vid = cv2.VideoCapture(0)
global ret, frame
"""
#YOLOv5 Model
model = torch.hub.load('ultralytics/yolov5', 'yolov5n')
#detection phase
global cl
while True:
    # Capture the video frame
    ret, frame = vid.read()
    rgb_image = frame[:, :, ::-1]
    results = model([rgb_image],augment=False)
    res = results.xyxy[0][:,0:6]#the fifth element is confidence, sixth is class
    if(len(res)):  
        indsel = 0
        maxconf = 0
        for i, obj in enumerate(res):
            if(obj[4] > maxconf):
                maxconf = obj[4]
                indsel = i
        box = res[indsel]
        global x1,y1,x2,y2
        x1 = box[0]
        y1 = box[1]
        x2 = box[2]
        y2 = box[3]
        global p1, p2, p3, p4
        p1 = (x1, y1)
        p2 = (x2, y1)
        p3 = (x2, y2)
        p4 = (x1, y2)
        cl=int(box[5])
        print(cl)
        cv2.polylines(frame,np.int32([np.array([p1,p2,p3,p4])]),True,(255,0,0),1)
        #cv2.putText(frame,str(cl),p4,cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1,2,)
        break
"""
# Capture the video frame
ret, frame = vid.read()
frame = imutils.resize(frame, width=300)
#now we have the object to track
(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')
# Set up tracker
tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN', 'MOSSE', 'CSRT']
tracker_type = tracker_types[7]

if int(minor_ver) < 3:
    tracker = cv2.Tracker_create(tracker_type)
else:
    if tracker_type == 'BOOSTING':
        tracker = cv2.legacy.TrackerBoosting_create()
    if tracker_type == 'MIL':
        tracker = cv2.TrackerMIL_create()
    if tracker_type == 'KCF':
        tracker = cv2.TrackerKCF_create()
    if tracker_type == 'TLD':
        tracker = cv2.legacy.TrackerTLD_create()
    if tracker_type == 'MEDIANFLOW':
        tracker = cv2.TrackerMedianFlow_create()
    if tracker_type == 'GOTURN':
        tracker = cv2.TrackerGOTURN_create()
    if tracker_type == 'MOSSE':
        tracker = cv2.legacy.TrackerMOSSE_create()
    if tracker_type == "CSRT":
        tracker = cv2.TrackerCSRT_create()
# Define an initial bounding box
#bbox = (int(x1),int(y1),int(x2)-int(x1),int(y2)-int(y1))
# Uncomment the line below to select a different bounding box
bbox = cv2.selectROI(frame, False)
# Initialize tracker with first frame and bounding box
ret = tracker.init(frame, bbox)
dist = 0
#PID PARAMETERS
KP = 0.2
KD = 0.5
KI = 0.001
prev_err = 0
derivative = 0
integral = 0
time_dif = 0
prev_t = time.time()
while True:
    dist = Get_Distance()
    if(dist <= 22): break
    # Capture the video frame
    ret, frame = vid.read()
    frame = imutils.resize(frame, width=300)
    # Update tracker
    ret, bbox = tracker.update(frame)
    # Draw bounding box
    if ret:
        # Tracking success
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
        #cv2.putText(frame,str(cl),p2,cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1,2,)
        pcx = bbox[0]+float(bbox[2])/2 #center
        err = pcx/150-1 #from -1 to 1
        #PID ADJUSTMENT CALCULATION
        time_dif = time.time()-prev_t
        integral += err#*time_dif
        derivative = (err-prev_err)#/time_dif
        action = err*KP + derivative*KD + integral*KI
        prev_err = err
        print("Err: "+str(err*KP)+" Der: "+str(derivative*KD)+" Int: "+str(integral*KI))
        #print(coef)
        print("Distance: "+str(dist))
        #cv2.putText(frame, "Action: "+str(action), (50,100), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),1,2)
        Move(min(max(action,-1),1),20)
        prev_t = time.time()
    else :
        # Tracking failure
        #cv2.putText(frame, "Tracking failure", (50,100), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),1,2)
        print("Tracking failure")
        Motor_Stop()
        exit()
    #cv2.putText(frame,"Distance: "+str(Get_Distance())+"cm",(50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),1,2)
    # Display the resulting frame
    cv2.imshow('frame', frame)
    # the 'q' button is set as the
    # quitting button
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
print("Distance reached "+str(dist))
Motor_Stop()
time.sleep(1)
dist = Get_Distance()
hand_move(dist-2)
time.sleep(1)
Move(0,15)
time.sleep(1.5)
Motor_Stop()
SetServoAngle(4,90)
time.sleep(0.5)
catched()
time.sleep(1)
# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()
