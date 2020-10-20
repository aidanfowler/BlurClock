import cv2
import numpy as np
import sys
import subprocess
import time
import psutil

# https://github.com/Itseez/opencv/blob/master/data/haarcascades/haarcascade_frontalface_default.xml
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades+"haarcascade_frontalface_default.xml")
# https://github.com/Itseez/opencv/blob/master/data/haarcascades/haarcascade_eye.xml
eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades+"haarcascade_eye_tree_eyeglasses.xml")

cap = cv2.VideoCapture(0)

lastEyeDetectionTime = 0
clockIsOn = False
clockSubprocess = subprocess.Popen(['echo','starting program'])
clockSubprocess.terminate()

while 1:
    ret, img = cap.read()
    #uncomment to flip video 
    #img = cv2.flip(img,0)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
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
            
            lastEyeDetectionTime = int(round(time.time()*1000))
            if(clockIsOn==False):
                print ("Found Eyes, Turn On Screen")
                clockSubprocess = subprocess.Popen(['sudo','../../rpi-rgb-led-matrix/examples-api-use/clock','--led-slowdown-gpio=4','--led-gpio-mapping=adafruit-hat-pwm','--led-rows=32','--led-cols=64','--led-brightness=50', '-f','../../rpi-rgb-led-matrix/fonts/8x13B.bdf','-d','%I:%M:%S','-y','10','-C','255,255,255'])
            clockIsOn = True
    cv2.imshow('Face', img)
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break
    if(int(round(time.time()*1000)) - lastEyeDetectionTime > 1000):
       if(clockIsOn):
           print ("it has been two seconds since eyes were last detected, turn off screen")
           process = psutil.Process(clockSubprocess.pid)
           for proc in process.children(recursive=True):
               proc.kill()
           process.kill()
           clockIsOn = False
cap.release()
cv2.destroyAllWindows()
