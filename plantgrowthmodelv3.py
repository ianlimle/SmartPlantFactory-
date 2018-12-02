# -*- coding: utf-8 -*-
"""
Created on Sun Aug  26 00:08:52 2018

@author: Ian
""" 

# import the necessary packages
from keras.preprocessing.image import img_to_array   #used to preprocess input frames 
from keras.models import load_model                  #load model from disk
from pyimagesearch.utils.simple_obj_det import image_pyramid
from pyimagesearch.utils.simple_obj_det import sliding_window
from pyimagesearch.utils.simple_obj_det import classify_batch
from imutils.object_detection import non_max_suppression                                                  
from imutils.video import VideoStream, FPS
from multiprocessing import Process
import numpy as np
import subprocess
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
    # define the paths to the Keras deep learning model 
    MODEL_PATH = "plantgrowthv2.model"
    
    #INPUT_SIZE: the width and height of our input to whcih the image is resized prior to being fed into the CNN 
    INPUT_SIZE = (400, 400)
    #PYR_SCALE: the scale of our image pyramid
    PYR_SCALE = 1.5
    #WIN_STEP: step of our sliding window
    WIN_STEP = 8
    #ROI_SIZE: input ROI size to our CNN as if we were perforimg classification
    ROI_SIZE = (96, 96)
    #BATCH_SIZE: size of batch to be passed through the CNN
    BATCH_SIZE = 32
    
    # load the model
    print("loading model...")
    model = load_model(MODEL_PATH)
    
    #initialize the object detection dictionary which maps class labels to their predicted bounding boxes and associated probability
    labels = {"Futterkohl_Gruner_Ring":[], "Lollo_Rossa":[], "Mangold_Lucullus":[], "Pak_Choi":[], "Tatsoi":[]}

    # initialize the video stream and allow the camera sensor to warm up
    print("starting video stream...")
    #vs = VideoStream(src=0).start()
    vs = VideoStream(usePiCamera=True).start()
    time.sleep(2.0)
    fps = FPS().start()

    # loop over the frames from the video stream
    while fps._numFrames<100:
        # grab the frame from the threaded video stream and resize it
        # to be a square
        frame = vs.read()
        (h, w) = frame.shape[:2]
        frame = cv2.resize(frame, INPUT_SIZE, interpolation=cv2.INTER_CUBIC)
        
        #initialize the batch ROIs and (x,y)-coordinates
        batchROIs = None
        batchLocs = []
        
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
        
        #start the timer
        print("detecting plants...")
        
        # loop over the image pyramid
        for image in image_pyramid(frame, scale= PYR_SCALE, minSize= ROI_SIZE):
            # loop over the sliding window locations
            for (x, y, roi) in sliding_window(image, WIN_STEP, ROI_SIZE):
                #take the ROI and preprocess it so we can classify the region with Keras
                roi = img_to_array(roi)
                roi = np.expand_dims(roi, axis=0)
                
                #if batchROIs is None, initialize it
                if batchROIs is None:
                    batchROIs = roi
                    
                #otherwise add the ROI to the bottom of the batch
                else:
                    batchROIs = np.vstack([batchROIs, roi])
                
                # append the (x,y)-coordinates of the sliding window to the batch
                batchLocs.append((x,y))
        
        #check to see if the batch is full
        if len(batchROIs) == BATCH_SIZE: 
            #classify the batch, then reset the batch ROIs and (x,y)-coordinates
            labels = classify_batch(model, batchROIs, batchLocs, labels, minProb=0.5)
        
        #reset the batch ROIs and (x,y) -coordinates
        batchROIs = None
        batchLocs = []
        
        #check to see if there are any remaining ROIs that still need to be classified
        if batchROIs is not None:
            labels = classify_batch(model, batchROIs, batchLocs, labels, minProb=0.5)
        
        #loop over all the class labels for each detected object in the image
        for classLabel in labels.keys():
            #grab the bounding boxes and associated probabilities for each detection and apply non-maxima suppression 
            #to suppress weaker overlapping detections
            boxes = np.array(p[0] for p in labels[classLabel])
            proba = np.array(p[1] for p in labels[classLabel])
            boxes = non_max_suppression(boxes, proba)
            
            #loop over the bounding boxes again, this time only drawing the ones that were not suppressed
            for (xA, yA, xB, yB) in boxes:
                title = "{}: {:.2f}%".format(classLabel, proba* 100)
                
                #if str(classLabel) == "Lollo_Rossa": 
                cv2.rectangle(frame, (xA, yA), (xB, yB), (255,0,255), 2)
                frame = cv2.putText(frame, title, (xA, yB), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)              
                
                #elif str(classLabel) == "Tatsoi":
                #    cv2.rectangle(frame, (xA, yA), (xB, yB), (255,0,0), 2)
                #    frame = cv2.putText(frame, title, (xA, yB), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0), 2)
                    
                    
            if len(labels[classLabel]) > 5:
                # send a text notification to firebase for app to receive
                triggered("Young seedling has matured. Time to transplant"+str(classLabel))
    
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
        
    