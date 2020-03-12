import os
import random
import fnmatch
import datetime
import pickle

# data processing
import numpy as np
np.set_printoptions(formatter={'float_kind':lambda x: "%.4f" % x})

import pandas as pd
pd.set_option('display.width', 300)
pd.set_option('display.float_format', '{:,.4f}'.format)
pd.set_option('display.max_colwidth', 200)

# tensorflow
import tensorflow as tf
import keras
from keras.models import Sequential  # V2 is tensorflow.keras.xxxx, V1 is keras.xxx
from keras.layers import Conv2D, MaxPool2D, Dropout, Flatten, Dense
from keras.optimizers import Adam
from keras.models import load_model
print( f'tf.__version__: {tf.__version__}' )
print( f'keras.__version__: {keras.__version__}' )

# sklearn
from sklearn.utils import shuffle
from sklearn.model_selection import train_test_split

# imaging
import cv2
from imgaug import augmenters as img_aug
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from PIL import Image


def prep_paths(path, file, num):
	with open(file) as f:
		raw_headings = f.readlines()
	f.close()
	imgs = []
	headings = []
	for i in range(num):
		headings.append(int(raw_headings[i]))
		imgs.append(path + str(i) + '.jpg')
	return imgs, headings

def image_preprocess(image):
	height, _, _ = image.shape
	image = image[int(height/2):,:,:]  # remove top half of the image, as it is not relevant for lane following
	image = cv2.cvtColor(image, cv2.COLOR_RGB2YUV)  # Nvidia model said it is best to use YUV color space
	image = cv2.GaussianBlur(image, (3,3), 0)
	image = cv2.resize(image, (200,66)) # input image size (200,66) Nvidia model
	image = image / 255 # normalizing
	return image
	
def image_data_generator(image_paths, steering_angles, batch_size):
	while True:
		batch_images = []
		batch_steering_angles = []

		for i in range(batch_size):
			random_index = random.randint(0, len(image_paths) - 1)
			image_path = image_paths[random_index]
			image = cv2.imread(image_paths[random_index])
			steering_angle = steering_angles[random_index]

			image = image_preprocess(image)
			batch_images.append(image)
			batch_steering_angles.append(steering_angle)

		yield( np.asarray(batch_images), np.asarray(batch_steering_angles))

def nvidia_model():
	model = Sequential(name='Nvidia_Model')
	
	# elu=Expenential Linear Unit, similar to leaky Relu
	# skipping 1st hiddel layer (nomralization layer), as we have normalized the data
	
	# Convolution Layers
	model.add(Conv2D(24, (5, 5), strides=(2, 2), input_shape=(66, 200, 3), activation='elu')) 
	model.add(Conv2D(36, (5, 5), strides=(2, 2), activation='elu')) 
	model.add(Conv2D(48, (5, 5), strides=(2, 2), activation='elu')) 
	model.add(Conv2D(64, (3, 3), activation='elu')) 
	model.add(Dropout(0.2)) # not in original model. added for more robustness
	model.add(Conv2D(64, (3, 3), activation='elu')) 

	# Fully Connected Layers
	model.add(Flatten())
	model.add(Dropout(0.2)) # not in original model. added for more robustness
	model.add(Dense(100, activation='elu'))
	model.add(Dense(50, activation='elu'))
	model.add(Dense(10, activation='elu'))

	# output layer: turn angle (from 45-135, 90 is straight, <90 turn left, >90 turn right)
	model.add(Dense(1)) 

	# since this is a regression problem not classification problem,
	# we use MSE (Mean Squared Error) as loss function
	optimizer = Adam(lr=1e-3) # lr is learning rate
	model.compile(loss='mse', optimizer=optimizer)

	return model
	
model = nvidia_model()
print(model.summary())

images, headings = prep_paths('images/img', 'headings.txt', 3)

X_train, X_valid, y_train, y_valid = train_test_split(images, headings, test_size=0.2)
print("Training data: %d\nValidation data: %d" % (len(X_train), len(X_valid)))

checkpoint_callback = tf.keras.callbacks.ModelCheckpoint(filepath='lane_navigation_check.h5', verbose=1, save_best_only=True)

history = model.fit_generator(image_data_generator( X_train, y_train, batch_size=100),
                              steps_per_epoch=300,
                              epochs=10,
                              validation_data = image_data_generator( X_valid, y_valid, batch_size=100),
                              validation_steps=200,
                              verbose=1,
                              shuffle=1,
                              callbacks=[checkpoint_callback])
# always save model output as soon as model finishes training
model.save('lane_navigation_final.h5')

result = model.predict(X_valid)
print("prediction result: ", result)
