# -*- coding: utf-8 -*-
"""
Created on Fri Oct 26 18:17:25 2018

@author: Ian
"""

from keras.models import Sequential
from keras.layers.normalization import BatchNormalization
from keras.layers.convolutional import Conv2D
from keras.layers.convolutional import MaxPooling2D
from keras.layers.core import Activation
from keras.layers.core import Flatten
from keras.layers.core import Dropout
from keras.layers.core import Dense
from keras import backend as K

class AlexNet:
    @staticmethod
    def build(width, depth, height, classes):
        model = Sequential()
        inputShape = (height, width, depth)
        chanDim =-1
        
        if K.image_data_format()== "channels_last":
            inputShape = (depth, height, width)
            chanDim = 1
            
        model.add(Conv2D(96, (11,11), padding=0, stride=4, input_shape=inputShape))
        model.add(Activation("relu"))
        model.add(BatchNormalization(axis=chanDim))
        model.add(MaxPooling2D(pool_size=(3,3), stride=2))
        
        model.add(Conv2D(256, (5,5), padding=2, stride=1))
        model.add(Activation("relu"))
        model.add(BatchNormalization(axis=chanDim))
        model.add(MaxPooling2D(pool_size=(3,3), stride=2))
        
        model.add(Conv2D(384, (3,3), padding=1, stride=1))
        model.add(Activation("relu"))
        model.add(BatchNormalization(axis=chanDim))
        model.add(Conv2D(384, (3,3), padding=1, stride=1))
        model.add(Activation("relu"))
        model.add(BatchNormalization(axis=chanDim))
        model.add(Conv2D(256, (3,3), padding=1, stride=1))
        model.add(Activation("relu"))
        model.add(BatchNormalization(axis=chanDim))
        model.add(MaxPooling2D(pool_size=(3,3), stride=2))
        
        model.add(Flatten())
        model.add(Dense(4096))
        model.add(Activation("relu"))
        model.add(BatchNormalization(axis=chanDim))
        model.add(Dropout(0.5))
        
        model.add(Flatten())
        model.add(Dense(4096))
        model.add(Activation("relu"))
        model.add(BatchNormalization(axis=chanDim))
        model.add(Dropout(0.5))
        
        model.add(Dense(classes))
        model.add(Activation("softmax"))
        
        return model
        
        