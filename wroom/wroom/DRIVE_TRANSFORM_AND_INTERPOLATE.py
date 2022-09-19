from math import sqrt, degrees, atan
from numpy import interp
import numpy as np

def lerp( x, min_in, max_in, min_out, max_out ):
	return min_out + (x - min_in) * (max_out-min_out)/(max_in-min_in)

	def bezier_interp(self, x, limits, delta):
		reverse, neutral, forward = np.array([
			[-2, limits[0]],
			[0, (sum(limits)/len(limits)) + delta],
			[2, limits[1]]
			])
		P = lambda t: (1-t)**2 * reverse[1] + 2*t*(1-t) * neutral[1] + t**2 * forward[1]
		return P(x)

	def lazy_ramp(self, x, y, acc):
		alpha = y + x
		beta = y - x
		alpha_unit = interp(alpha, [-2, 2], [-1, 1])
		beta_unit = interp(beta, [-2, 2], [-1, 1])
		acc = interp(acc, [-1, 1], [1, 10])
		alpha_final = alpha ** acc
		self.publish_serial(alpha_final, beta_unit)


	def simple_alpha_beta(self, x, y, max_speed):
		alpha = y+x
		beta = y-x
		xp = interp(max_speed, [-1,1], [-10,10])
		alpha_lerp = self.bezier_interp(alpha, [127, 1], max_speed)
		beta_lerp = self.bezier_interp(beta, [128, 255], max_speed)
		self.publish_serial(alpha_lerp, beta_lerp)

	def harmonic_alpha_beta(alpha, beta):
			xp, yp = [1, 127], [128, 255]
			alpha_lerp = interp(alpha, [-1, 1], xp)
			beta_lerp = interp(beta, [-1, 1], yp)
			self.publish_serial(alpha_lerp, beta_lerp)

	def calculate_alpha_beta(self, x, y, delta):
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
			self.lazy_ramp(alpha, beta)
