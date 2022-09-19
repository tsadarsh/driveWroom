from math import sqrt, degrees, atan
from numpy import interp
import numpy as np


def simple_alpha_beta(joy):
	x = joy.axes[0]
	y = joy.axes[1]

	alpha = y-x
	beta = y+x
	alpha_lerp = interp(alpha, [-2,2], [1, 127])
	beta_lerp = interp(beta, [-2, 2], [128, 255])

	return [alpha_lerp, beta_lerp] 
