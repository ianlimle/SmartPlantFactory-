# SmartPlantFactory-
This is an implementation of a semi-autonomous plant factory system which is able to self-regulate
1. Temperature and Humidity
2. pH
3. Water refill
4. Lighting (LEDs)

Besides these, it has additional plant health monitoring and growth tracking capabilities. Both of which employed the use of OpenCV, Tensorflow and Keras to implement. The Convolutional Neural Network trained was able to classify 5 different crop types with an accuracy of 70 % (could still be improved). 

Arduino Mega 2560 and Raspberry Pi Model 3B+ and Pi Camera were used. 

# Model Architecture
The CNN architecture (smallervggnet.py) used here is a smaller compact variant of the VGGNet network characterised by:
1. Using only 3x3 convo layers stacked on top one another in increasing depth
2. Reduce volume size by max pooling 
3. Fully-connected layers at the end of a nextwork prior to a softmax layer

# Usage
## To train the neural network
1. Run python train.py --dataset name_of_your_input_dataset --model name_of_your_output_model_file --labelbin name_of_your_output_label_binarizer_file on your terminal

Running the above outputs:
1. model file (that stores your model's weights) 
2. pickle file (that stores the binarized labels for your classes)
3. png image of training process (plot.png)

## To load the model, label binarizer file

2. Run python plantgrowthmodelv3.py on Raspberry Pi 
