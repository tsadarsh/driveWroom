# Generic ROS imports
import serial
import struct
import rclpy
from rclpy.node import Node
import os
print(os.getcwd())
# ROS messages imports
from wroom.msg import sabertooth_simplified_serial as sss
from sensor_msgs.msg import Joy

# Local imports
import DRIVE_TRANSFORM_INTERPOLATE as DTI


class JoyToDrive(Node):
	"""Initialise node that subscribes to /joy topic and sends the joy 
	values to DTI. The DTI computes the velocity commands for motors.
	Motor commands are published in /motor topic.
	"""

	def __init__(self):
		motor_cmd = [] # unit8 array

		super().__init__ ('joy_subscriber')
		self.ser = 0
		self.start_serial()
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
		motor_cmd = DTI.affine_tranform(joy)
		self.get_logger().info("{motor_cmd}")

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

	joy_subscriber = JoySubscriber()
	#motor_driver_cmd = MotorDriverCMD()

	rclpy.spin(joy_subscriber)

	joy_subscriber.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
