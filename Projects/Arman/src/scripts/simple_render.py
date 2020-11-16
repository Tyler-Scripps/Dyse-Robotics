import numpy as np
from Arm import Arm, R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

def get_axis(pose):
	origin = pose[:3]
	x = origin + R(np.array([3,0,0]), ref=pose[3:])
	y = origin + R(np.array([0,3,0]), ref=pose[3:])
	z = origin + R(np.array([0,0,3]), ref=pose[3:])
	return origin, x, y, z

def draw(arman):
	ax = plt.subplot(111, projection='3d')
	ax.axes.set_xlim3d(left=-20, right=20) 
	ax.axes.set_ylim3d(bottom=-20, top=20) 
	ax.axes.set_zlim3d(bottom=0, top=40) 
	origins = [[0],[0],[0]]
	for pose in arman.joint_poses:
		o, x, y, z = get_axis(pose)
		origins[0].append(o[0])
		origins[1].append(o[1])
		origins[2].append(o[2])
		x = list(zip(o,x))
		y = list(zip(o,y))
		z = list(zip(o,z))
		ax.plot(x[0], x[1], x[2], 'b')
		ax.plot(y[0], y[1], y[2], 'r')
		ax.plot(z[0], z[1], z[2], 'g')
	ax.plot(origins[0], origins[1], origins[2], 'c')
	plt.show()

def main():
	arman = Arm(pose=[90,135,45,0,0], sim=True)
	for i in range(100):
		arman.move_to([0,0,0,90])
		if i % 10 == 0:
			draw(arman)
		print(arman.wp_as_string())

	plt.show()


if __name__=="__main__":
	main()