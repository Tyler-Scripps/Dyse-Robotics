import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
import imutils
from matplotlib.collections import LineCollection

def nothing(x):
	pass

# read/display original image
image = cv.cvtColor(cv.imread('images/img1.jpg'), cv.COLOR_BGR2RGB)
image = image[int(image.shape[0]/3):image.shape[0],0:image.shape[1]]
base_image = image.copy()
fig, ((ax1, ax2, ax3, ax4), (ax5, ax6, ax7, ax8)) = plt.subplots(2, 4, figsize=(15,7))
hsv = cv.cvtColor(image, cv.COLOR_RGB2HSV)
blurred = cv.bilateralFilter(image, 10, 75, 75)
#blurred_hsv = cv.GaussianBlur(hsv,(5,5),0)
#blurred_hsv = cv.medianBlur(gray, 5)
blurred_hsv = cv.cvtColor(blurred, cv.COLOR_RGB2HSV)
#cv.adaptiveThreshold(gray,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY,11,2)

ax1.imshow(blurred)
ax2.imshow(blurred_hsv)
ax1.set_title('blurred')
ax2.set_title('blurred_hsv')

#cv.namedWindow("View")
#cv.createTrackbar('R', 'View', 0, 255, nothing)
#cv.createTrackbar('G', 'View', 0, 255, nothing)
#cv.createTrackbar('B', 'View', 0, 255, nothing)
#cv.createTrackbar('Upper/Lower', 'View', 0, 1, nothing)

"""
while 1:
	upper = np.array([230, 200, 120])
	lower = np.array([150, 120, 0])

	mask = cv.inRange(image, lower, upper)
	seg_img = cv.bitwise_and(image, image, mask=mask)

	cv.imshow('View', seg_img)
	cv.waitKey()

	if cv.getTrackbarPos('Upper/Lower', 'View'):
		upper[0] = cv.getTrackbarPos('R', 'View')
		upper[1] = cv.getTrackbarPos('G', 'View')
		upper[2] = cv.getTrackbarPos('B', 'View')
	else:
		lower[0] = cv.getTrackbarPos('R', 'View')
		lower[1] = cv.getTrackbarPos('G', 'View')
		lower[2] = cv.getTrackbarPos('B', 'View')
"""
low_hsv = np.array([20,0,100])
upp_hsv = np.array([95,85,140])

l_hsv = np.array([20,0, 100])
h_hsv = np.array([95,150, 250])

low_rgb = np.array([140, 140, 70])
upp_rgb = np.array([255, 255, 245])

mask1 = cv.inRange(blurred_hsv, l_hsv, h_hsv)
mask2 = cv.inRange(blurred_hsv, low_hsv, upp_hsv)
ax3.set_title('hsv mask')
ax4.set_title('noise mask')
ax3.imshow(mask1)
ax4.imshow(mask2)
mask = mask1 ^ mask2
seg_img = cv.bitwise_and(blurred_hsv, blurred_hsv, mask=mask)
ax5.set_title('segmented')
ax5.imshow(seg_img)

"""
# plots a scatter of the colors in the photo
r, g, b = cv.split(image)
fig = plt.figure()
axis1 = fig.add_subplot(2, 1, 1, projection='3d')

pixel_colors = image.reshape((np.shape(image)[0]*np.shape(image)[1], 3))
norm = colors.Normalize(vmin=-1., vmax=1.)
norm.autoscale(pixel_colors)
pixel_colors = norm(pixel_colors).tolist()

axis1.scatter(r.flatten(), g.flatten(), b.flatten(), facecolors=pixel_colors, marker='.')
axis1.set_xlabel("red")
axis1.set_ylabel("green")
axis1.set_zlabel("blue")

hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
h, s, v = cv.split(hsv)
axis2 = fig.add_subplot(2, 1, 2, projection='3d')
axis2.scatter(h.flatten(), s.flatten(), v.flatten(), facecolors=pixel_colors, marker='.')
axis2.set_xlabel("hue")
axis2.set_ylabel("sat")
axis2.set_zlabel("val")

plt.show()
cv.waitKey()
"""
# convert to grayscale and display original
#cv.imshow("gray", gray)
#cv.waitKey()

# filter for edges and display
edges = cv.Canny(seg_img, 0, 500)
ax6.set_title('canny')
ax6.imshow(edges)

# read contours from grayscale
#contours, hierarchy = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
#contours = imutils.grab_contours(contours)
#contours = sorted(contours, key=cv.contourArea, reverse = True)[:3]
#box = cv.convexHull(contours[0])
"""
for c in contours:
	peri = cv.arcLength(c, True)
	approx = cv.approxPolyDP(c, 0.015 * peri, True)
	# if our approximated contour has four points, then
	# we can assume that we have found our screen
	if len(approx) == 4:
		boundary = approx
		break
"""
# draws the contours on the original
#cv.drawContours(image, contours[0], -1, (0,255,0), 3)
#cv.imshow("Segmented Contours", image)
#cv.waitKey()
line_img = np.zeros((image.shape[0], image.shape[1],3), dtype=np.uint8,)

# function calculates and draws lines in the edge image
threshold = 0
min_lin_len = 500
max_lin_gap = 0
dX = 0
dY = 0
line_col = []
lines = cv.HoughLinesP(edges, 100, 1, threshold, min_lin_len, max_lin_gap)
for line in lines:
	for x1, y1, x2, y2 in line:
		cv.line(line_img, (x1, y1), (x2, y2), [0, 255, 0], 2)
		dX += (x2-x1)
		dY += (y2-y1)
		
		line_col.append([(x1,-y1),(x2,-y2)])
lineCollection = LineCollection(line_col)
print('delta sums:',dX,dY)
dX = dX / len(lines)
dY = dY / len(lines)
print('deltas:',dX,dY)
print('num lines:',len(lines))
image = cv.addWeighted(image, 0.8, line_img, 1.0, 0.0)
# draw indicator line
center_point = (int(image.shape[1]/2),int(image.shape[0]/2))
end_point = (center_point[0] - int(20 * dY), center_point[1] - int(20 * dX))
print('center:',center_point)
print('end_point:',end_point)
cv.line(image, center_point, end_point, (255,0,0), 2)
# mark front of indicator
cv.circle(image, end_point, 1, (0,255,0), -1)

ax7.set_title('processed')
ax7.imshow(image)
ax8.set_title('original')
ax8.imshow(base_image)
plt.show()
cv.waitKey()
#fig, ax = plt.subplots()
#ax.add_collection(lineCollection)
#ax.autoscale()
#plt.show()
cv.destroyAllWindows()
