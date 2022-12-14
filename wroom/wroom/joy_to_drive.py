import numpy as np

# Generic ROS imports
import struct
import rclpy
from rclpy.node import Node

# ROS messages imports
from wroom_msgs.msg import SabertoothSerial as sss
from sensor_msgs.msg import Joy

# Local imports
import wroom.DRIVE_TRANSFORM_INTERPOLATE as DTI


class JoyToDrive(Node):
	"""Initialise node that subscribes to /joy topic and sends the joy 
	values to DTI. The DTI computes the velocity commands for motors.
	Motor commands are published in /motor topic.
	"""

	def __init__(self):
		self.motor_cmd = [] # unit8 array

		super().__init__('joy_to_drive')
		self.ser = 0
		#self.start_serial()
		self.subscription = self.create_subscription(
			Joy,
			'joy',
			self.DTI_caller,
			10)
		self.subscription

		# Publisher to publish motor commands 
		self.publisher_ = self.create_publisher(sss, 'motor', 10)
		timer_period = 0.5 # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)

	def DTI_caller(self, joy):
		"""Sends joy input to DTI and logs returned values"""
		self.motor_cmd = DTI.affine_tranform(joy)
		self.get_logger().info(f"{self.motor_cmd}")

	def timer_callback(self):
		msg = sss()
		msg.cmd = self.motor_cmd
		self.publisher_.publish(msg)

	# def start_serial(self):
	# 	try:
	# 		ser_dev = serial.Serial('/dev/ttyTHS1', baudrate=9600)
	# 		self.ser = ser_dev
	# 		self.get_logger().info("Serial COM connection established")
	# 	except:
	# 		self.get_logger().info("Serial COM unsuccessful")

	# def publish_serial(self, alpha, beta):
	# 	self.ser.write(struct.pack('>B', int(alpha)))
	# 	self.ser.write(struct.pack('>B', int(beta)))
	# 	self.get_logger().info(f'data: {alpha}, {beta}')

# class MotorDriverCMD(Node):

# 	def __init__(self):
# 		super().__init__('motor_driver_cmd')
# 		self.publisher_ = self.create_publisher(sss)
# 		timer_period = 0.5 # seconds
# 		self.timer = self.create_timer(timer_period, self.timer_callback)


def main(args=None):
	rclpy.init(args=args)

	joy_subscriber = JoyToDrive()
	#motor_driver_cmd = MotorDriverCMD()

	rclpy.spin(joy_subscriber)

	joy_subscriber.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
