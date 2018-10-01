# -*- coding: utf-8 -*-
"""
Created on Mon Jul 16 11:52:21 2018

@author: Ian
"""
import requests
import json
import cv2
import time
import subprocess
import numpy as np
from imutils.video import FPS
from imutils.video import WebcamVideoStream
from datetime import datetime
from threading import Timer

def triggered(message):
   #url = 'https://bosch-ville-api.unificationengine.com/v1/message/send'
   #api_token = 'Y2gmZGV2aWNlX3R5cGU9WERJ'
   #headers = {'Content-Type': 'application/json', 'Authorization': api_token}
   #body = {"phone_number": "+6590698810", "message":message}
   #requests.post(url, data=json.dumps(body), headers=headers)
    
   firebase_url= "https://slv-plantfactory.firebaseio.com/"
   location = "Growth state"
   headers = "Date: " + time.strftime("%Y%m%d") + ", " + "Time: " + time.strftime("%H:%M:%S")
   body = str(message)
   requests.put(firebase_url + location + "/" + headers + ".json", data=json.dumps(body))

def process_frame():
    vs = WebcamVideoStream(src=0).start()
    time.sleep(2.0)
    fps = FPS().start()
    
    while fps._numFrames <1000:
    #capture frame-by-frame
        img = vs.read()  
        
        #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        #a typical green will have still have some red and blue 
        #we will get the lower-light mixes of all colors still 
        lower_green = np.array([30,100,100])
        upper_green = np.array([90,255,255])
    
        #create a mask for a specific range
        #pixels in the range will be converted to pure white 
        #pixels outside the range will be converted to black
        mask = cv2.inRange(hsv, lower_green, upper_green)
    
        #restore 'green-ness' by running a bitwise operation 
        #show color when there is the frame AND the mask
        res = cv2.bitwise_and(img,img, mask= mask)
        
        #load the young seedling cascade file
        #youngseedling_cascade = cv2.CascadeClassifier('youngseedling.xml')
        #initialize the young seedling object as a list of rectangle coordinates 
        #youngseedling = youngseedling_cascade.detectMultiScale(gray,50,50)
        
        #for (x,y,w,h) in youngseedling:
            #cv2.rectangle(img,(x,y),(x+h,y+h),(255,0,0),2)
            #print(youngseedling)
            #font = cv2. FONT_HERSHEY_SIMPLEX
            #cv2.putText(img, 'Wrist', (w-x,y-h), font, 0.5,(11,255,255), 2, cv2.LINE_AA)
            #triggered("Young seedling has sprouted. Time for transplant.")
        
        #load the mature seedling cascade file
        #matureseedling_cascade = cv2.CascadeClassifier('matureseedling.xml')
        #initialize the young seedling object as a list of rectangle coordinates 
        #matureseedling = matureseedling_cascade.detectMultiScale(gray,50,50)
        
        #for (x,y,w,h) in matureseedling:
            #cv2.rectangle(img,(x,y),(x+h,y+h),(255,0,0),2)
            #print(matureseedling)
            #font = cv2. FONT_HERSHEY_SIMPLEX
            #cv2.putText(img, 'Wrist', (w-x,y-h), font, 0.5,(11,255,255), 2, cv2.LINE_AA)
            #triggered("Young seedling has matured. Time for transplant.")

        #cv2.imshow('img', img)
        #cv2.imshow('mask', mask)
        #cv2.imshow('res', res)
        
        date_yyyymmdd = time.strftime("%Y-%m-%d")
        #saves 5 color filtered images in JPG format in the working directory for post
        for index in range(1,6):
            cv2.imwrite(str(date_yyyymmdd)+"_pic_"+str(index)+".jpg", res)
        
        #waitKey():a keyboard binding function
        #its argument is the time in milliseconds
        #it waits for a specified milliseconds for any keyboard event
        #if you press any key in that time, the program continues
        if cv2.waitKey(1) & 0xFF ==ord('q'):
            break
        
        fps.update()
        
    fps.stop()        
    cv2.destroyAllWindows()
    vs.stop()        

now = datetime.today()
run_at = now.replace(day=now.day, hour=8, minute=48, second=0, microsecond=0)
delta_t = run_at-now

secs = delta_t.seconds+1

if __name__== "__main__":
    subprocess.Popen("raspivid -o - -t 0 -n -w 640 -h 480 -fps 30 | cvlc -vvv stream:///dev/stdin --sout '#standard{access=http,mux=ts,dst=:8554}' :demux=h264", shell=True) 
    t = Timer(secs, process_frame)
    t.start()
    
    
    
    