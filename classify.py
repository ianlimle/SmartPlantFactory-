# USAGE
# python classify.py --model plantgrowth.model --labelbin label_bin_plantgrowth.pickle --image C:/Users/Ian/Desktop/lolla.jpg

# import the necessary packages
from keras.preprocessing.image import img_to_array
from keras.models import load_model
import numpy as np
import argparse
import imutils
import pickle
import cv2
import os

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-m", "--model", required=True,
                help="path to trained model")
ap.add_argument("-l", "--labelbin", required=True,
                help="path to label binarizer")
ap.add_argument("-i", "--image", required=True,
                help="path to input image")
args = vars(ap.parse_args())

# load the image
image = cv2.imread(args["image"])
#make a copy of the image called output for display purposes
output = image.copy()
 
# pre-process the image for classification
image = cv2.resize(image,(96, 96))
image = image.astype("float") / 255.0
image = img_to_array(image)
image = np.expand_dims(image, axis=0)

# load the trained convolutional neural network and the label binarizer file
print("loading network...")
model = load_model(args["model"])
lb = pickle.loads(open(args["labelbin"], "rb").read())

# classify the input image
print("classifying image...")
#calculate probabilities
proba = model.predict(image)[0]
idx = np.argmax(proba)
#create the label
label = lb.classes_[idx]

# mark prediction as "correct" of the input image filename
# assuming input image has a filename that contains the true label
# extract name of the plant from the filename 
filename = args["image"][args["image"].rfind(os.path.sep) + 1:]
#compare it to the label 
correct = "correct" if filename.rfind(label) != -1 else "incorrect"

# build the label and draw the label on the image
label = "{}: {:.2f}% ({})".format(label, proba[idx] * 100, correct)
output = imutils.resize(output, width=400)
cv2.putText(output, label, (10, 25),  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

# show the output image
print("{}".format(label))
cv2.imshow("Output", output)
cv2.waitKey(0)