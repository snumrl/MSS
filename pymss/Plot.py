import math
import random
import time
import os
import sys
from datetime import datetime

import collections
from collections import namedtuple
from collections import deque
from itertools import count

import numpy as np
from IPython import embed
import matplotlib
import matplotlib.pyplot as plt

plt.ion()

def Plot(y,title,num_fig=1,ylim=True):
	# temp_y = np.zeros(y.shape)
	# if y.shape[0]>5:
	# 	temp_y[0] = y[0]
	# 	temp_y[1] = 0.5*(y[0] + y[1])
	# 	temp_y[2] = 0.3333*(y[0] + y[1] + y[2])
	# 	temp_y[3] = 0.25*(y[0] + y[1] + y[2] + y[3])
	# 	for i in range(4,y.shape[0]):
	# 		temp_y[i] = np.sum(y[i-4:i+1])*0.2

	plt.figure(num_fig)
	plt.clf()
	# plt.hold(True)
	plt.title(title)
	plt.plot(y,'b')
	
	# plt.plot(temp_y,'r')

	plt.show()
	if ylim:
		plt.ylim([0,1])
	plt.pause(0.001)

