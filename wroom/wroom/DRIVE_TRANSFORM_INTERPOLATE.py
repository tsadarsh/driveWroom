from math import sqrt, degrees, atan
from numpy import interp
import numpy as np

def affine_tranform(joy):
	input_s = np.array([joy.axes[0], joy.axes[1], 1])
	tfm = np.array([[1, 63, 64],[0.5, 63.5, 191.5],[0,0,1]])
	output_s = np.matmul(input_s, np.transpose(tfm))
	return [int(i) for i in output_s[0:2].tolist()]

def simple_alpha_beta(joy):
	x = joy.axes[0]
	y = joy.axes[1]

	alpha = y-x
	beta = y+x
	alpha_lerp = interp(alpha, [-2,2], [1, 127])
	beta_lerp = interp(beta, [-2, 2], [128, 255])

	return [alpha_lerp, beta_lerp] 
