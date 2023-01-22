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
from norfair import Detection, Tracker, Video, draw_tracked_objects, draw_boxes

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

####################################################
##Function name：Get_Distance()
##Function performance ultrasonic ranging，return distance(unit is centimeter）
##Entrance parameter：none
##Exit parameter：none
####################################################
def	Get_Distance():
    time.sleep(0.1)
    GPIO.output(TRIG,GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(TRIG,GPIO.LOW)
    while not GPIO.input(ECHO):
                pass
    t1 = time.time()
    while GPIO.input(ECHO):
                pass
    t2 = time.time()
    time.sleep(0.1)
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

limits = {}
limits[1] = [0,180]#hand 1
limits[2] = [20,180]#hand 2
limits[3] = [0,180]#hand 3
limits[4] = [90,180]#hand 4
limits[7] = [0,150]#camera 1
limits[8] = [0,180]#camera 2

prev = {}
prev[1] = 180
prev[2] = 20
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

time.sleep(2)
servo_init()
#YOLOv5 Model
model = torch.hub.load('ultralytics/yolov5', 'yolov5n')
# define a video capture object
vid = cv2.VideoCapture(0)
#detection phase
global cl
while True:
    # Capture the video frame
    global ret, frame
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
tracker = Tracker(distance_function="iou", distance_threshold=0.7)
# Define an initial bounding box
#bbox = (int(x1),int(y1),int(x2)-int(x1),int(y2)-int(y1))
# Uncomment the line below to select a different bounding box
bbox = cv2.selectROI(frame, False)
# Initialize tracker with first frame and bounding box
p1 = [int(bbox[0]), int(bbox[1])]
p2 = [int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3])]
global norfair_detections
norfair_detections = [Detection(np.array([p1,p2]))]
tracked_objects = tracker.update(detections=norfair_detections)
draw_boxes(frame, tracked_objects)
while True:
    # Capture the video frame
    ret, frame = vid.read()
    rgb_image = frame[:, :, ::-1]
    results = model([rgb_image],augment=False)
    res = results.xyxy[0][:,0:6]#the fifth element is confidence, sixth is class
    norfair_detections.clear()
    for i, obj in enumerate(res):
        x1 = obj[0]
        y1 = obj[1]
        x2 = obj[2]
        y2 = obj[3]
        norfair_detections.append(Detection(np.array([[x1,y1],[x2,y2]])))
    tracked_objects = tracker.update(detections=norfair_detections)
    draw_boxes(frame,norfair_detections)
    draw_boxes(frame, tracked_objects)
    cv2.putText(frame,"Distance: "+str(Get_Distance())+"cm",(50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),1,2)
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
