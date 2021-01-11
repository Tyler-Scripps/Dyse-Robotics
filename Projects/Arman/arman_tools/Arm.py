import numpy as np
import matplotlib.pyplot as plt
from Transformer import Transformer
from mpl_toolkits.mplot3d import Axes3D

class Arm:
	def __init__(self, init_joint_poses=None, motor_spin=[1,1,-1,-1,1], restrictions=None, default_joint_angles=[90, 135, 45, 0, 90]):
		if init_joint_poses is None:
			# values are approximations for development purposes
			self.init_joint_poses = np.array([[0, 0, 2, np.pi/2, 0, 0],
											   [11.5, 0, 0, 0, 0, np.pi],
											   [0, -5.75, 0, 0, 0, 3 * np.pi / 4],
											   [-6, -3.5, 0, np.pi, -np.pi/2, 0]
											  ]) 
		else:
			self.init_joint_poses = init_joint_poses
			
		# evironment dimensions
		self.envDim = 3
		# number of joints
		self.nJoints = len(self.init_joint_poses) + 1
		
		# an empty set of transformers
		self.autobots = []
		
		self.spin = motor_spin
		
		# initialize state constraints and objects
		self.j_rstr = np.zeros((self.nJoints, 2))
		self.poseActual = np.zeros((self.nJoints))
		self.joint_poses = np.zeros((self.nJoints, self.envDim))
		
		self.set_restrictions(restrictions)
		self.adjust_joints(default_joint_angles)
		
		# update kinematics
		self.forward_IK()
		
	def build_transformers(self):
		# the base joint is defined as the world origin
		self.autobots = []
		prev_tf = None
		for i,joint in enumerate(self.init_joint_poses):
			trns = Transformer(0.0, 0.0, np.radians(self.poseActual[i]) * self.spin[i], 'temp', protocol=['psi'])  
			translationActual = trns.transform(joint[:3], inverse=True)
			tf = Transformer(joint[3], joint[4], joint[5] + np.radians(self.poseActual[i]) * self.spin[i], f'Joint{i+1}',translation=translationActual, parent=prev_tf)
			if not prev_tf is None:
				self.autobots[-1].child = tf
			self.autobots.append(tf)
			prev_tf = tf
		
	def set_restrictions(self, restrictions):
		if restrictions is None:
			# default Arman joint restrictions
			assert 5 == self.nJoints, 'set_restrictions requested: failed, number of joints is not 5(default)'
			self.j_rstr = np.array([[0, 180],[45, 135],[0, 135],[0, 135],[0, 180]])
		else:
			assert len(restrictions) == self.nJoints, f'set_restrictions requested: failed, restrictions set length {len(restrictions)} != {self.nJoints}'
			self.j_rstr = restrictions
	
	def forward_IK(self):
		# update the transformers
		self.build_transformers()
		# Calculate the new joint poses.
		# Passing [0.0,0.0,0.0] to transform
		# will return the origin of that frame 
		# in parent coords.
		for i in range(self.nJoints-1, 0, -1):
			tf = self.autobots[i-1]
			point = np.array([0.0, 0.0, 0.0])
			while not tf is None:
				point = tf.transform(point, inverse=True)
				tf = tf.parent
			self.joint_poses[i] = point
			
	def adjust_joints(self, adjustments):
		assert len(adjustments) == self.nJoints, f'Angle adjustment requested: joint angle set shape mismatch {np.array(adjustments).shape} -> {self.nJoints}'
		# make sure no angle adjustments contradict valid configurations
		for i,joint in enumerate(self.poseActual):
			self.poseActual[i] = max(self.j_rstr[i][0], min(joint + adjustments[i], self.j_rstr[i][1]))
		self.forward_IK()
		
	def set_joints(self, poses):
		assert len(poses) == self.nJoints, f'Angle set requested: joint angle set shape mismatch {np.array(pose).shape} -> {self.nJoints}'
		# make sure no angle adjustments contradict valid configurations
		for i in range(self.nJoints):
			self.poseActual[i] = max(self.j_rstr[i][0], min(poses[i], self.j_rstr[i][1]))
		self.forward_IK()

class Renderer:
	def __init__(self):
		self.verbose = True
		self.visualize = True

	def draw(self, joint_poses):
		ax = plt.subplot(111, projection='3d')
		ax.axes.set_xlim3d(left=-20, right=20) 
		ax.axes.set_xlabel('X')
		ax.axes.set_ylim3d(bottom=-20, top=20) 
		ax.axes.set_ylabel('Y')
		ax.axes.set_zlim3d(bottom=0, top=40) 
		origins = [[0],[0],[0]]
		for pose in joint_poses:
			origins[0].append(pose[0])
			origins[1].append(pose[1])
			origins[2].append(pose[2])
			ax.scatter(pose[0], pose[1], pose[2], color='r', marker='x')
		ax.plot(origins[0], origins[1], origins[2], color='c', alpha=0.6)
		plt.show()
