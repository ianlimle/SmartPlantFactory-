# -*- coding: utf-8 -*-
"""
Created on Mon Sep 17 22:36:41 2018

@author: Ian
"""

import pickle
import imutils
import numpy as np


def sliding_windows(image, step, ws):
    #slide a window across the image
    for y in range(0, image.shape[0] - ws[1], step):
        for x in range(0, image.shape[1] - ws[0], step):
            #yield the current window
            yield (x, y, image[y:y + ws[1], x:x + ws[0]])
            
def image_pyramid(image, scale=1.5, minSize=(96,96)) :
    #yield the original image
    yield image

    #keep looping over the image pyramid
    while True:
        #compute the dimensions of the next image in the pyramid
        w = int(image.shape[1] / scale)
        image = imutils.resize(image, width=w)
        
        #if resized image does not meet the supplied minimum size, stop constructing the image
        if image.shape[0] < minSize[1] or image.shape[1] < minSize[0]:
            break
        
        #yield the next image in the pyramid
        yield image
        
def classify_batch(model, batchROIs, batchLocs, labels, minProb=0.5, top=10, dims=(96,96)):
    # pass out batch ROIs through the network and decode the predictions
    proba = model.predict(batchROIs)
    idx = np.argmax(proba)
    LABELBIN_PATH = "label_bin_plantgrowth.pickle"
    lb = pickle.loads(open(LABELBIN_PATH, "rb").read())
    label = lb.classes_(idx)
    
    if proba > minProb:
        (pX, pY)= batchLocs
        box = (pX, pY, pX+dims[0], pY+dims[1])
        
        #grab the list of predictions for the label and 
        #add the bounding box coordinates and associated class label probability to the list
        L = labels.get(label, [])
        L.append((box, proba))
        labels[label] = L
        
    #return the labels dictionary    
    return labels    
        
    
    
    