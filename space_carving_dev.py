"""
	Author: Mitchell Scott
	Project: 3D reconstruction by space carving
	Date: Oct 20, 2020

		This is a concept implementation of space carving.
	The goal of this project is to use 2D monocular images to 
	reconstruct an object. 
	*This version attempt is purely 
	theoretical and the success is not a proof of concept*

"""

import cv2 as cv
import numpy as np 
import matplotlib.pyplot as plt 


IMAGE_RESOLUTION = (640, 480, 3)


def get_projection(object):
	pass


def main():
	blank_image = np.zeros((480, 640, 3), np.uint8)
	start_point = (int(2 * IMAGE_RESOLUTION[0] / 5), int(2 * IMAGE_RESOLUTION[1] / 5))
	end_point = (int(3 * IMAGE_RESOLUTION[0] / 5), int(3 * IMAGE_RESOLUTION[1] / 5))
	square_image = cv.rectangle(blank_image, start_point, end_point, (0,0,255), -1)

	cv.imshow('Object', square_image)
	plt.imshow(square_image)
	plt.show()
	#cv.waitKey()


if __name__ == "__main__":
	main()
	cv.destroyAllWindows()