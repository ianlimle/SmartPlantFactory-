# -*- coding: utf-8 -*-
"""
Created on Thu Aug  9 00:08:34 2018

@author: Ian
"""
#CNN architecture used is a smaller compact variant of the VGGNet network characterised by:
#1. Using only 3x3 convo layers stacked on top one another in increasing depth
#2. Reduce volume size by max pooling 
#3. Fully-connected layers at the end of a nextwork prior to a softmax layer

# import the necessary packages
from keras.models import Sequential
from keras.layers.normalization import BatchNormalization
from keras.layers.convolutional import Conv2D
from keras.layers.convolutional import MaxPooling2D
from keras.layers.core import Activation
from keras.layers.core import Flatten
from keras.layers.core import Dropout
from keras.layers.core import Dense
from keras import backend as K

class SmallerVGGNet:
    @staticmethod
    def build(width, height, depth, classes):
        # initialize the model along with the input shape to be
        # "channels last" and the channels dimension itself
        model = Sequential()
        inputShape = (height, width, depth)
        chanDim = -1
        
        # if we are using "channels first", update the input shape and channels dimension
        if K.image_data_format() == "channels_first":
            inputShape = (depth, height, width)
            chanDim = 1
            
        # first CONV => RELU => CONV => RELU => POOL layer set
        model.add(Conv2D(32, (3, 3), padding="same",      #convo layer has 32 filters with 3x3 kernel 
                         input_shape=inputShape))                       
        model.add(Activation("relu"))                     #use RELU activation function followed by
        model.add(BatchNormalization(axis=chanDim))       #BatchNormalization layer to zero-centre the activations
        model.add(MaxPooling2D(pool_size=(3, 3)))
        model.add(Conv2D(32, (3, 3), padding="same"))      #convo layer has 32 filters with 3x3 kernel                       
        model.add(Activation("relu"))                     #use RELU activation function followed by
        model.add(BatchNormalization(axis=chanDim))         #use a 3x3 pool size to reduce spatial dimensions quickly from 96x96 down to 32x32
        model.add(Dropout(0.25))                          #utilize dropout: randomly disconnect nodes from POOL layer that have a probability of 25% during training  
        
        # second CONV => RELU => CONV => RELU => POOL layer set
        model.add(Conv2D(64, (3, 3), padding="same"))
        model.add(Activation("relu"))
        model.add(BatchNormalization(axis=chanDim))
        model.add(Conv2D(64, (3, 3), padding="same"))
        model.add(Activation("relu"))
        model.add(BatchNormalization(axis=chanDim))
        model.add(MaxPooling2D(pool_size=(2, 2)))
        model.add(Dropout(0.25))
        
        # (CONV => RELU) * 3 => POOL
        model.add(Conv2D(128, (3, 3), padding="same"))
        model.add(Activation("relu"))
        model.add(BatchNormalization(axis=chanDim))
        model.add(Conv2D(128, (3, 3), padding="same"))
        model.add(Activation("relu"))
        model.add(BatchNormalization(axis=chanDim))
        model.add(Conv2D(128, (3, 3), padding="same"))
        model.add(Activation("relu"))
        model.add(BatchNormalization(axis=chanDim))
        model.add(MaxPooling2D(pool_size=(2, 2)))
        model.add(Dropout(0.25))
        
        # first (and only) set of FC => RELU layers
        model.add(Flatten())
        model.add(Dense(1024))
        model.add(Activation("relu"))
        model.add(BatchNormalization())
        model.add(Dropout(0.5))
        
        # softmax classifier
        model.add(Dense(classes))
        model.add(Activation("softmax"))
        
        # return the constructed network architecture
        return model