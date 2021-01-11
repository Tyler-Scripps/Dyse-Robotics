import board
import rospy
import busio
from adafruit_servokit import ServoKit
from arman_controller.srv import ArmanDriverRequest, ArmanDriverRequestResponse

i2c_addr = busio.I2C(board.SCL, board.SDA)
kit = ServoKit(channels=16, i2c=i2c_addr)

def update_joints(req):
	kit.servo[0].angle = req.joint0
	kit.servo[1].angle = req.joint1
	kit.servo[2].angle = req.joint2
	kit.servo[3].angle = req.joint3
	kit.servo[4].angle = req.joint4
	return ArmanDriverRequestResponse(1)

if __name__=='__main__':
	rospy.init_node('arman_driver')
	s = rospy.service('arman_driver', ArmanDriverRequest, update_joints)
	rospy.spin()
