# -*- coding: utf-8 -*-
"""
Created on Thu Aug  9 00:10:25 2018

@author: Ian
"""
# USAGE
# python train.py --dataset dataset --model plantgrowthv2.model --labelbin label_bin_plantgrowthv2.pickle

# set the matplotlib backend so figures can be saved in the background
import matplotlib
matplotlib.use("Agg")

# import the necessary packages
from keras.preprocessing.image import ImageDataGenerator           #used for data augmentation on existing images in the dataset; prevents overfitting
from keras.optimizers import Adam                                  #optimizer method used to train the network
from keras.preprocessing.image import img_to_array               
from sklearn.preprocessing import LabelBinarizer                   #allows input of class labels, 
                                                                   #transform our class labels into one-hot encoded vectors 

import keras                                                       #allow us to take an integer class label prediction from our Keras CNN and transform it back into a human-readable label 
from sklearn.model_selection import train_test_split               #create training and tests splits
from pyimagesearch.smallervggnet import SmallerVGGNet
import matplotlib.pyplot as plt                                    #
from imutils import paths
import numpy as np
import argparse
import random
import pickle
import cv2
import os

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-d", "--dataset", required=True,
                help="path to input dataset (i.e., directory of images)")
ap.add_argument("-m", "--model", required=True,
                help="path to output model")
ap.add_argument("-l", "--labelbin", required=True,
                help="path to output label binarizer")
ap.add_argument("-p", "--plot", type=str, default="plot.png",
                help="path to output accuracy/loss plot")
args = vars(ap.parse_args())

# initialize the number of epochs to train for, initial learning rate,
# batch size, and image dimensions
EPOCHS = 100                              #how many times our network "sees" each training example and learns patterns from it                     
INIT_LR = 1e-3                            #default learning rate for the Adam optimizer
BS = 3                                    #batch size
IMAGE_DIMS = (96, 96, 3)                  #spatial dimensions of the input images: 96 x 96 pixels with 3 channels(RGB)  

# initialize the data and labels as empty lists to store preprocessed images and labels 
data = []
labels = []

# grab the image paths and randomly shuffle them
print("loading images...")
imagePaths = sorted(list(paths.list_images(args["dataset"])))
random.seed(42)
random.shuffle(imagePaths)

# loop over the input images
for imagePath in imagePaths:
    # load the image, pre-process it, and store it in the data list
    image = cv2.imread(imagePath, cv2.IMREAD_COLOR)
    image = cv2.resize(image, (IMAGE_DIMS[1], IMAGE_DIMS[0]))
    #call Keras img_to_array function to convert image to a Keras-compatible array
    image = img_to_array(image)
    data.append(image)
    
    # extract the class label from the image path and update the labels list
    label = imagePath.split(os.path.sep)[-2]
    labels.append(label)

# convert data array to numpy array, scale the raw pixel intensities to the range [0, 1]
data = np.array(data, dtype="float") / 255.0
#convert labels from a list to a numpy array 
labels = np.array(labels)
print("data matrix: {:.2f}MB".format(data.nbytes / (1024 * 1000.0)))

# partition the data into training and testing splits using 80% of
# the data for training and the remaining 20% for testing
(trainX, testX, trainY, testY) = train_test_split(data, labels, test_size=0.2, random_state=42)

# binarize the labels
lb = LabelBinarizer()
#labels = lb.fit_transform(labels)
trainY = lb.fit_transform(trainY)
testY = lb.transform(testY)

# construct the image generator for data augmentation
aug = ImageDataGenerator(rotation_range=25, width_shift_range=0.1,
                         height_shift_range=0.1, shear_range=0.2, zoom_range=0.2,
                         horizontal_flip=True, fill_mode="nearest")

# initialize the model
print("compiling model...")
#initialize the Keras CNN model with 96 x96 x 3 input spatial dimensions 
model= SmallerVGGNet.build(width=IMAGE_DIMS[1], height=IMAGE_DIMS[0],
                            depth=IMAGE_DIMS[2], classes=len(lb.classes_))

#initialize Adam optimizer with learning rate decay 
opt = Adam(lr=INIT_LR, decay=INIT_LR / EPOCHS)
#compile with categorical cross-entropy as the loss if there are more than 2 classes
#compile with binary cross-entropy as the loss if there are only two classes  
model.compile(loss="binary_crossentropy", optimizer=opt, metrics=["accuracy"])

# train the network
print("training network...")
#trainY = keras.utils.np_utils.to_categorical(trainY)
#testY = keras.utils.np_utils.to_categorical(testY)

H = model.fit_generator(
        aug.flow(trainX, trainY),
        validation_data=(testX, testY),
        steps_per_epoch=len(trainX) // BS,
        epochs=EPOCHS, verbose=1)

# save the model to disk
print("serializing network...")
model.save(args["model"])

# save the label binarizer to disk
print("serializing label binarizer...")
f = open(args["labelbin"], "wb")
f.write(pickle.dumps(lb))
f.close()

# plot the training loss and accuracy
plt.style.use("ggplot")
plt.figure()
N = EPOCHS
plt.plot(np.arange(0, N), H.history["loss"], label="train_loss")
plt.plot(np.arange(0, N), H.history["val_loss"], label="val_loss")
plt.plot(np.arange(0, N), H.history["acc"], label="train_acc")
plt.plot(np.arange(0, N), H.history["val_acc"], label="val_acc")
plt.title("Training Loss and Accuracy")
plt.xlabel("Epoch #")
plt.ylabel("Loss/Accuracy")
plt.legend(loc="upper left")
plt.savefig(args["plot"])