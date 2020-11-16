import time
import numpy as np
from Arm import Arm, R


def main():

	cycle_t = 0.05
	arman = Arm(pose=[90,135,45,0,0])

	for i in range(100):
		start = time.time()

		# move arman to a pose
		arman.move_to([0,0,0,90])

		print(arman.wp_as_string())

		total_t = time.time() - start

		# normalize the cycle time
		if (cycle_t - total_t > 0):
			time.sleep(cycle_t - total_t)

if __name__=="__main__":
	main()
