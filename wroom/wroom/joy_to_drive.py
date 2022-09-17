from math import sqrt, degrees, atan
import serial
import struct
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16
from sensor_msgs.msg import Joy


def lerp( x, min_in, max_in, min_out, max_out ):
	return min_out + (x - min_in) * (max_out-min_out)/(max_in-min_in)

class JoySubscriber(Node):

	def __init__(self):
		super().__init__ ('joy_subscriber')
		self.ser = 0
		self.start_serial()
		self.subscription = self.create_subscription(
			Joy,
			'joy',
			self.listener_callback,
			10)
		self.subscription

	def start_serial(self):
		try:
			ser_dev = serial.Serial('/dev/ttyTHS1', baudrate=9600)
			self.ser = ser_dev
			self.get_logger().info("Serial COM connection established")
		except:
			self.get_logger().info("Serial COM unsuccessful")


	def listener_callback(self, joy):
		self.calculate_alpha_beta(-joy.axes[0], joy.axes[1])

	def calculate_alpha_beta(self, x, y):
		if x==0 and y==0:
			# joystick in home position
			alpha, beta = 0, 0
		elif x == 0:
			# joy in only forward or backward
			alpha, beta = y, y

		elif y == 0:
			alpha, beta = y, -y

		elif x > 0 and y > 0:
			# Quad 1
			alpha = sqrt(x**2 + y**2)
			beta = (y**2 - x**2)/(y**2 + x**2)
		elif x < 0 and y > 0:
			# Quad 2
			alpha = (y**2 - x**2)/(y**2 + x**2)
			beta = sqrt(x**2 + y**2)
		elif x < 0 and y < 0:
			# Quad 3
			alpha = -((y**2 - x**2)/(y**2 + x**2))
			beta = -(sqrt(x**2 + y**2))
		elif x > 0 and y < 0:
			alpha = -(sqrt(x**2 + y**2))
			beta = -((y**2 - x**2)/(y**2 + x**2))
		else:
			self.get_logger().info(f"unmapped data: {x}, {y}")
		self.get_logger().info(f'data: {alpha}, {beta}')
		if self.ser:
			alpha_lerp = lerp(alpha, -1, 1, 1, 127)
			beta_lerp = lerp(betam -1, 1, 128, 255)
			self.ser.write(struct.pack('>B', alpha_lerp))
			self.ser.write(struct.pack('>B', beta_lerp))

def main(args=None):
	rclpy.init(args=args)

	joy_subscriber = JoySubscriber()

	rclpy.spin(joy_subscriber)

	joy_subscriber.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
