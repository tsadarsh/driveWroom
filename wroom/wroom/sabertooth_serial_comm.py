import time
import serial
import struct

import rclpy
from rclpy.node import Node

from wroom_msgs.msg import SabertoothSerial as sss


PORT_1 = '/dev/ttyTHS1'
PORT_2 = '/dev/ttyTHS2'

class SendToMotorDriver(Node):
	def __init__(self):
		super().__init__('send_to_motor_driver')
		self.port_1 = self.prepare_port(addr=PORT_1)
		self.port_2 = self.prepare_port(addr=PORT_2)

		self.subscription = self.create_subscription(
			sss,
			'motor',
			self.serial_write,
			10)
		self.subscription

	def prepare_port(self, addr):
		try:
			port = serial.Serial('/dev/ttyTHS1', baudrate=9600)
			self.get_logger().info("Serial communication established on Port 1")
			return port_1

		except serial.serialutil.SerialException:
			self.get_logger().warn(f"Could not establish serial comm in {addr}")
			time.sleep(1) # seconds
			self.prepare_port(addr)

	def serial_write(self, data):
		drive_left = data.cmd[0]
		drive_right = data.cmd[1]
		try:
			self.port_1.write(struct.pack('>B', drive_left))
			self.port_1.write(struct.pack('>B', drive_right))
		except serial.serialutil.SerialException:
			self.get_logger().warn(f"Connection to {PORT_1} BROKEN! Trying again.")
			self.prepare_port(PORT_1)

		# try:
		# 	self.port_2.write(struct.pack('>B', drive_left))
		# 	self.port_2.write(struct.pack('>B', drive_right))
		# except serial.serialutil.SerialException:
		# 	self.get_logger().info(f"Connection to {PORT_1} BROKEN! Trying again.")
		# 	self.prepare_port(PORT_1)

def main(args=None):
	rclpy.init(args=args)

	send_to_motor_driver = SendToMotorDriver()

	rclpy.spin(send_to_motor_driver)

	send_to_motor_driver.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
