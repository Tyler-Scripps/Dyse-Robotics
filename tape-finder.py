import time
import smbus
import cv2 as cv
import streamer
import numpy as np
from picamera import PiCamera
import matplotlib.pyplot as plt
from picamera.array import PiRGBArray
from matplotlib.collections import LineCollection

# this bound captures the noise in hsv
# masked with the tape bounds eliminates 
# almost all noise
LOW_HSV = np.array([20,0,100])
UPP_HSV = np.array([95,85,140])

# tape bounds
# bound has significant noise
L_HSV = np.array([20,0, 100])
U_HSV = np.array([95,150, 250])

# default i2c address
# must be the same on arduino
i2c_add = 0x04
# enables the car to drive
# set to 0 when target located
drive_enabled = 1
# helps in the case where
# drive_heading is infinite
# i.e. dX is 0
DIRECTIONAL_ERR = .5


# finds the hough lines in the canny image
# also returns a mask with lines, matplotlib
# LineCollection and average dX, dY.
# TODO: split average m into clusters and
# 		map overlaping lines with cv.draw
def get_houghLines(edges):
	# tinker values
	threshold = 0
	min_lin_len = 200
	max_lin_gap = 0
	dX = 0
	dY = 0
	line_col = []
	# initalize blank image
	line_img = np.zeros((edges.shape[0], edges.shape[1],3), dtype=np.uint8,)
	lines = cv.HoughLinesP(edges, 1, .1, threshold, min_lin_len, max_lin_gap)
	for line in lines:
		for x1, y1, x2, y2 in line:
			# draw line on blank image
			cv.line(line_img, (x1, y1), (x2, y2), [0, 0, 255], 2)
			# summing the change in x and y
			dX += (x2-x1)
			dY += (y1-y2)
			# adds line to the collection
			line_col.append([(x1,-y1),(x2,-y2)])
	lineCollection = LineCollection(line_col)
	# divides the sum of deltas by number of lines
	# thus the average
	dX = dX / len(lines)
	dY = dY / len(lines)
	return [line_img, lines, lineCollection, dX, dY]


# draws the lines on the base image
# also draws the lane heading
def draw_lines(base_imag, lines):
	indicator_length = 20
	# combines the base and lines
	base_image = cv.addWeighted(base_image, 0.8, lines[0], 1.0, 0.0)
	# draw indicator line
	center_point = (int(base_image.shape[1]/2),int(base_image.shape[0]/2))
	end_point = (center_point[0] - int(indicator_length * lines[3]), center_point[1] - int(indicator_length * lines[4]))
	cv.line(base_image, center_point, end_point, (255,0,0), 2)
	# mark front of indicator
	cv.circle(base_image, end_point, 1, (0,255,0), -1)
	return base_image


# applies the base filters to perform 
# image segmentation, canny and draws the hough lines
# returns the processed image and lines data
def process_image(base_image):
	# bilateral filter blurs noise and perserves edges
	blurred = cv.bilateralFilter(base_image, 10, 75, 75)
	blurred_hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
	mask1 = cv.inRange(blurred_hsv, LOW_HSV, UPP_HSV)
	mask2 = cv.inRange(blurred_hsv, L_HSV, U_HSV)
	mask = mask1 ^ mask2
	seg_img = cv.bitwise_and(blurred_hsv, blurred_hsv, mask=mask)
	edges = cv.Canny(seg_img, 0, 500)
	lines_data = get_houghLines(edges)
	processed_img = draw_lines(base_img, lines_data)
	return [processed_img, lines_data]
	
	
# converts lane headings into wheel speeds
# wheel speeds are percentages and must be positive
# talk to Mitch about vizualizing the 
# TODO: update equations to better direct the bot
#		possibilities for this are using a neural net,
#		this will improve the precision and eleminate 
#		a lot of image processing
#		
def update_wheel_speeds(dX, dY, bus):
	if np.abs(dX) < .5:
		bus.write_i2c_block_data(i2c_add, drive_enabled, [100, 100, 1])
	direction = dY / dX
	dH = np.sqrt(dX**2 + dY**2)
	# positive m
	if direction > 0:
		l_wheel = 1
		# heading offset more than 45 to the right
		if direction < 1:
			drive_state = 2
			r_wheel = dX / dH
		else:
			drive_state = 1
			r_wheel = (dY - dX) / dH
	# negative m
	elif direction < 0:
		r_wheel = 1
		# heading offset more than 45 to the left
		if direction < -1:
			drive_state = 0
			l_wheel = dX / dH
		else:
			drive_state = 1
			l_wheel = (dY - dX) / dH
	else:
		drive_state = 1
		r_wheel, l_wheel = 1
	# send state to bus
	bus.write_i2c_block_data(i2c_add, drive_enabled, [l_wheel, r_wheel, drive_state])


def main():
	# initialize the bus
	bus = smbus.SMBus(1)
	# initailize the camera
	print('Camera Opening')
	camera = PiCamera()
	ca = (640, 480)
	camera.framerate = 32
	rawCapture = PiRGBArray(camera, size=(640, 480))
	time.sleep(.5)
	print('Ready')
	# initialize a stream to monitor the camera/processed
	#output = streamer.StreamingOutput(resolution='640x480', framerate=32)
	#addres = ('', 8000)
	#server = streamer.StreamingServer(address, StreamingHandler)
	#server.serve_forever()
	
	for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
		print("New Frame")
		base_image = frame.array
		#cv.imwrite('bare-stream.mjpg', base_image)
		data = processes_image(base_image)
		#cv.imwrite('proc-stream.mjpg', data[0])
		#fig, ax = plt.subplots()
		#ax.add_collection(data[1][2])
		#plt.show()
		update_wheel_speeds(data[1][3], data[1][4], bus)
		rawCapture.truncate(0)
		
if __name__=="__main__":
	main()
		
		
		
		