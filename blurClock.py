import cv2
import numpy as np
import sys
import subprocess
import time
import psutil
import RPi.GPIO as GPIO
from time import sleep

#Class for linear actuator control
class StepperHandler():

    __CLOCKWISE = 1
    __ANTI_CLOCKWISE = 0

    def __init__(self, stepPin, directionPin, delay=0.208, stepsPerRevolution=200):

        # Configure instance
        self.CLOCKWISE = self.__CLOCKWISE
        self.ANTI_CLOCKWISE = self.__ANTI_CLOCKWISE
        self.StepPin = stepPin
        self.DirectionPin = directionPin
        self.Delay = delay
        self.RevolutionSteps = stepsPerRevolution
        self.CurrentDirection = self.CLOCKWISE
        self.CurrentStep = 0

        # Setup gpio pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.StepPin, GPIO.OUT)
        GPIO.setup(self.DirectionPin, GPIO.OUT)

    def Step(self, stepsToTake, direction = __CLOCKWISE):
        # Set the direction
        GPIO.output(self.DirectionPin, direction)
        # Take requested number of steps
        for x in range(stepsToTake):
            #print("Step " + str(x))
            GPIO.output(self.StepPin, GPIO.HIGH)
            self.CurrentStep += 1
            sleep(self.Delay)
            GPIO.output(self.StepPin, GPIO.LOW)
            sleep(self.Delay)
            

def init():
    #move linear actuator to very front andthen backwards
    #so we can set the correct starting position when clock turns on
    stepperHandler.Step(500,stepperHandler.ANTI_CLOCKWISE)
    #stepperHandler.Step(750)
    #turn on clock (using RTC & rpi-rgb-led-matrix clock)
    #clockSubprocess = subprocess.Popen(['sudo','../rpi-rgb-led-matrix/examples-api-use/clock','--led-slowdown-gpio=4','--led-gpio-mapping=adafruit-hat-pwm','--led-rows=32','--led-cols=64','--led-brightness=50', '-f','../rpi-rgb-led-matrix/fonts/8x13B.bdf','-d','%I:%M:%S','-y','10','-C','255,255,255'])
    #this subprocess runs from root folder from auto start in etc/rc.local
    clockSubprocess = subprocess.Popen(['sudo','home/pi/rpi-rgb-led-matrix/examples-api-use/clock','--led-slowdown-gpio=4','--led-gpio-mapping=adafruit-hat-pwm','--led-rows=32','--led-cols=64','--led-brightness=100', '-f','/home/pi/rpi-rgb-led-matrix/fonts/8x13B.bdf','-d','%I:%M:%S','-y','10','-C','255,255,255'])
#clockSubprocess = subprocess.Popen(['sudo','home/pi/rpi-rgb-led-matrix/examples-api-use/clock','--led-slowdown-gpio=4','--led-gpio-mapping=adafrui$
    
    
def end():
    process = psutil.Process(clockSubprocess.pid)
    for proc in process.children(recursive=True):
        proc.kill()
    process.kill()           
    cap.release()
    cv2.destroyAllWindows()
    
    
#trained modelsfor detecting face and open eyes
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades+"haarcascade_frontalface_default.xml")
eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades+"haarcascade_eye_tree_eyeglasses.xml")
#get webcam
cap = cv2.VideoCapture(0)
#count how long it has been since we have seen eyes
lastEyeDetectionTime = 0
clockIsOn = False
#instantiate subprocess for running rgb matrix program from python
clockSubprocess = subprocess.Popen(['echo','starting program'])
clockSubprocess.terminate()

# Define pins
STEP_PIN = 25
DIRECTION_PIN = 19

# Create a new instance of our stepper class (note if you're just starting out with this you're probably better off using a delay of ~0.1)
stepperHandler = StepperHandler(STEP_PIN, DIRECTION_PIN, 0.005)
init()

while 1:
    #get video frame
    ret, img = cap.read()
    #uncomment to flip video 
    #img = cv2.flip(img,0)
    #turn to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #find faces
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    ###Face detection
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
        roi_gray = gray[y:y + h, x:x + w]
        roi_color = img[y:y + h, x:x + w]
        ##Eye Detection
        eyes = eye_cascade.detectMultiScale(roi_gray)
        for (ex, ey, ew, eh) in eyes:
            cv2.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 0), 2)
            if(clockIsOn==False):
                print ("Found eyes, move screen forward")
                stepperHandler.Step(750,stepperHandler.ANTI_CLOCKWISE)
            lastEyeDetectionTime = int(round(time.time()*1000))
            clockIsOn = True
    
    if(int(round(time.time()*1000)) - lastEyeDetectionTime > 1500):
       if(clockIsOn):
           print ("it has been a second since eyes were last detected, move screen backwards")
           stepperHandler.Step(750)
           clockIsOn = False
           
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break
end()       


