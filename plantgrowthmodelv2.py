# -*- coding: utf-8 -*-
"""
Created on Sun Aug  26 00:08:52 2018

@author: Ian
""" 

# import the necessary packages
from keras.preprocessing.image import img_to_array   #used to preprocess input frames 
from keras.models import load_model                  #load model from disk
from imutils.video import VideoStream, FPS
from multiprocessing import Process
import numpy as np
import subprocess
import pickle
import imutils
import time
import cv2
import requests
import json

def triggered(message):
    firebase_url= "https://slv-plantfactory.firebaseio.com/"
    location = "Growth state"
    headers = "Date: " + time.strftime("%Y%m%d") + ", " + "Time: " + time.strftime("%H:%M:%S")
    body = str(message)
    requests.put(firebase_url + location + "/" + headers + ".json", data=json.dumps(body))          
                    
def plant_growth_health_tracking():
    # define the paths to the Keras deep learning model and label binarizer file
    MODEL_PATH = "plantgrowth.model"
    LABELBIN_PATH = "label_bin_plantgrowth.pickle"

    # initialize the total number of frames that *consecutively* contain
    # santa along with threshold required to trigger the growthchange state
    TOTAL_CONSEC = 0
    TOTAL_THRESH = 20

    # initialize whether the growthchange state has been triggered
    GrowthChange = False

    # load the model
    print("loading model...")
    model = load_model(MODEL_PATH)
    lb = pickle.loads(open(LABELBIN_PATH, "rb").read())

    # initialize the video stream and allow the camera sensor to warm up
    print("starting video stream...")
    #vs = VideoStream(src=0).start()
    vs = VideoStream(usePiCamera=True).start()
    time.sleep(2.0)
    fps = FPS().start()

    # loop over the frames from the video stream
    while fps._numFrames<100:
        # grab the frame from the threaded video stream and resize it
        # to have a maximum width of 400 pixels
        frame = vs.read()
        frame = imutils.resize(frame, width=400)
        
        #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
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
        res = cv2.bitwise_and(frame,frame, mask= mask)
        
        date_yyyymmdd = time.strftime("%Y-%m-%d")
        #saves 5 color filtered images in JPG format in the working directory for post
        for index in range(1,6):
            cv2.imwrite(str(date_yyyymmdd)+"_pic_"+str(index)+".jpg", res)
    
        # prepare the image to be classified by our deep learning network
        image = cv2.resize(frame, (96, 96))
        image = image.astype("float") / 255.0
        image = img_to_array(image)
        image = np.expand_dims(image, axis=0)
    
        # classify the input image and initialize the label and
        # probability of the prediction
        #calculate probabilities
        proba = model.predict(image)[0]
        idx = np.argmax(proba)
        #create the label
        label = lb.classes_[idx]
        
        # check to see if santa was detected using our convolutional neural network
        if proba[idx] > 0.8:
            # increment the total number of consecutive frames that contain the plant type
            TOTAL_CONSEC += 1
        
            # check to see if we should raise the growthchange state
            if not GrowthChange and TOTAL_CONSEC >= TOTAL_THRESH:
                # indicate that santa has been found
                GrowthChange = True
                
                # send a text notification to firebase for app to receive
                triggered("Young seedling has matured. Time for transplant.")
                
            # otherwise, reset the total number of consecutive frames and the growthchange state
            else:
                TOTAL_CONSEC = 0
                GrowthChange = False
        
        # build the label and draw it on the frame
        label = "{}: {:.2f}%".format(label, proba[idx] * 100)
        frame = cv2.putText(frame, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    
        # show the output frame
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF
        
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break
        
        fps.update()

    # do a bit of cleanup
    print("cleaning up...")
    fps.stop()
    cv2.destroyAllWindows()
    vs.stop()
    
if __name__ == "__main__":
    p= Process(target= plant_growth_health_tracking)
    while True:
        sub = subprocess.Popen("raspivid -o - -t 0 -n -w 640 -h 480 -fps 30 | cvlc -vvv stream:///dev/stdin --sout '#standard{access=http,mux=ts,dst=:8554}' :demux=h264"
                               , shell=True)
        if str(time.strftime("%H:%M:%S"))== "13:00:00":
            sub.terminate()
            p.start()
            p.join()
        
    