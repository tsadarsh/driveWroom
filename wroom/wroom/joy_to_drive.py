import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16
from sensor_msgs.msg import Joy


class JoySubscriber(Node):

	def __init__(self):
		super().__init__ ('joy_subscriber')
		self.subscription = self.create_subscription(
			Joy,
			'joy',
			self.listener_callback,
			10)
		self.subscription

	def listener_callback(self, joy):
		self.get_logger().info(f'data: {joy.axes}')

def main(args=None):
	rclpy.init(args=args)

	joy_subscriber = JoySubscriber()

	rclpy.spin(joy_subscriber)

	joy_subscriber.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
