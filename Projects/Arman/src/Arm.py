import numpy as np
from adafruit_servokit import ServoKit as SK

def R(effector, ref=[]):
	""" Rotates a 3D vector (effector) by some angles (ref)
		effector can be a 3D vector or a 6D pose
		if 6D only the x,y,z will be affected. phi, theta, psi need
		to be updated on their own
		ref must be a list of 3 angles to rotate about each axis
		default ref is poseActuals angular offsets"""
	R = np.matrix([
					[np.cos(ref[1])*np.cos(ref[2]),
					np.cos(ref[1])*np.sin(ref[2]),
					-np.sin(ref[1])
				],
					[np.sin(ref[0])*np.sin(ref[1])*np.cos(ref[2])-(np.cos(ref[0])*np.sin(ref[2])),
					np.sin(ref[0])*np.sin(ref[1])*np.sin(ref[2])+(np.cos(ref[0])*np.cos(ref[2])),
					np.sin(ref[0])*np.cos(ref[1])
				],
					[np.cos(ref[0])*np.sin(ref[1])*np.cos(ref[2])+(np.sin(ref[0])*np.sin(ref[2])),
					np.cos(ref[0])*np.sin(ref[1])*np.sin(ref[2])-(np.sin(ref[0])*np.cos(ref[2])),
					np.cos(ref[0])*np.cos(ref[1])
				]
			])

	if len(effector) > 3:
		return np.concatenate([np.array(np.matmul(R, np.array([effector[:3]]).T)).reshape(3), effector[3:] + ref])

	return np.array(np.matmul(R, effector.T)).reshape(3)

class Arm:
	def __init__(self, pose=[90, 145, 45, 90, 90], sim=False):
		self.sim = sim
		self.poseActual = np.array(pose)
		if not sim:
			self.kit = SK(channels=16)
			self.kit.servo[0].angle = self.poseActual[0]
			self.kit.servo[4].angle = self.poseActual[1]
			self.kit.servo[7].angle = self.poseActual[2]
			self.kit.servo[8].angle = self.poseActual[3]
			self.kit.servo[12].angle = self.poseActual[4]

		self.pose_restrictions = np.zeros((5,2))
		self.set_restrictions()

		self.links = np.array([9.5, 11.5, 7.5, 11.5, (6,3.5)], dtype=object)
		self.joint_poses = np.zeros((5,6))
		self.world_pose = np.zeros(4)
		self.solve_joint_IK()
		self.alpha = 2

	def wp_as_string(self):
		return f'({self.world_pose[0]}, {self.world_pose[1]}, {self.world_pose[2]}, {self.world_pose[3]})'

	def set_restrictions(self):
		self.pose_restrictions[0] = np.array([0, 180])
		self.pose_restrictions[1] = np.array([30, 150])
		self.pose_restrictions[2] = np.array([0, 135])
		self.pose_restrictions[3] = np.array([0, 135])
		self.pose_restrictions[4] = np.array([0, 180])

	def rotate_solution(self, joints):
		for i,joint in enumerate(joints):
			self.joint_poses[i] = R(joint, ref=np.array([0,0,-np.radians(self.poseActual[0])]))

	def solve_joint_IK(self):
		""" finds the world frame endpoint of the next link
			all calulations are done in the XZ plane
			then rotated about z appropriately

			TODO: update into matrix form, got to baked 
		"""
		# unrotated base joint is a constant
		solved_poses = [np.array([0, 0, self.links[0], 0, 0, 0])]
		alpha, beta, gamma, phi, theta = np.radians(self.poseActual) * [-1,1,1,1,1]
		
		x1 = 0
		z1 = self.links[1]
		solved_poses.append(np.array([x1, 0, z1, 0, beta, 0]))

		gamma += beta - (np.pi/2)
		x2 = self.links[2] * np.cos(beta) + solved_poses[-1][0]
		z2 = self.links[2] * np.sin(beta) + solved_poses[-1][2]
		solved_poses.append(np.array([x2, 0, z2, 0, gamma, 0]))

		phi += gamma - (np.pi/2)
		x3 = self.links[3] * np.cos(gamma) + solved_poses[-1][0]
		z3 = self.links[3] * np.sin(gamma) + solved_poses[-1][2]
		solved_poses.append(np.array([x3, 0, z3, 0, phi, 0]))

		x4 = (self.links[4][0] * np.cos(phi)) + (self.links[4][1] * np.sin(phi))  + solved_poses[-1][0]
		z4 = (self.links[4][0] * np.sin(phi)) + (self.links[4][1] * np.cos(phi))  + solved_poses[-1][2]
		solved_poses.append(np.array([x4, 0, z4, theta, phi, 0]))

		self.rotate_solution(solved_poses)

	def update_self(self):
		""" update poseActual to the motor poses
		"""
		r = np.sqrt(self.joint_poses[-1][0]**2 + self.joint_poses[-1][1]**2)
		self.world_pose = [r, self.joint_poses[-1][2], self.poseActual[0], self.poseActual[4]]

		if self.sim:
			return

		self.poseActual = np.array([
			self.kit.servo[0].angle,
			self.kit.servo[4].angle,
			self.kit.servo[7].angle,
			self.kit.servo[8].angle,
			self.kit.servo[12].angle])

	def update_phys(self):
		""" Update the motor poses to poseActaul
		"""
		if self.sim:
			return
		print(self.poseActual)
		self.kit.servo[0].angle = self.poseActual[0]
		self.kit.servo[4].angle = self.poseActual[1]
		self.kit.servo[7].angle = self.poseActual[2]
		self.kit.servo[8].angle = self.poseActual[3]
		self.kit.servo[12].angle = self.poseActual[4]

	def adjust_pose(self, target):
		"""step the motors toward a goal pose
		"""
		self.update_self()

		dp = self.alpha * np.sign(target - self.poseActual)

		for i,p in enumerate(self.poseActual):
			m = np.max((self.pose_restrictions[i][0], p + dp[i]))
			self.poseActual[i] = np.min((self.pose_restrictions[i][1], m))

		self.solve_joint_IK()

		self.update_phys()

	def control_height(self, h0):
		h_actual = self.joint_poses[-1][2]
		p = np.sign(h0 - h_actual)

		if p < 0:
			return np.array([180,0,45])
		else:
			return np.array([90,90,0])

	def move_to(self, target):
		# alpha and theta translate directly to the world frame
		c1 = self.control_height(target[1])
		self.adjust_pose([target[2], c1[0], c1[1], c1[2], target[3]])
