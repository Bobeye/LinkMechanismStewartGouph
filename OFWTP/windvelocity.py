import math
import random
import numpy as np
import matplotlib.pyplot as plt

Vwind_max = 9		# Maximum wind speed
Vwind_min = 4		# Minimum wind speed
Vwind_period = 100	# wind changing period / second

Vwind_map = [4.0,5.0,6.0,7.1,7.5,8.0,8.3,8.8]		# initialize wind map

def FIXED_WIND():
	return random.choice(Vwind_map)

def FIXED_CHANGE_WIND():
	Fixed_wind_list = ['1175', '1020', '890', '840', '800']
	return Fixed_wind_list

def CHANGE_WIND():
	Vary_wind_list = [8.0]
	for i in range(2*len(Vwind_map)):
		Vary_wind_list.append(random.choice(Vwind_map))
	return Vary_wind_list



