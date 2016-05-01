import math
import random
import numpy as np
import matplotlib.pyplot as plt

Vwind_max = 9		# Maximum wind speed
Vwind_min = 4		# Minimum wind speed
Vwind_period = 100	# wind changing period / second

Vwind_map = [6.2, 7.1, 8.0, 8.35, 8.85]		# initialize wind map

# wind speed and wind tunnel comment
Vwind_dict = {6.2: '1175', 7.1: '1020', 8.0: '890', 8.35: '840', 8.85: '800'}


def FIXED_WIND():
	return random.choice(Vwind_map)

def FIXED_CHANGE_WIND():
	Fixed_wind_list = [6.2, 7.1, 8.0, 8.35, 8.85, 8.35, 8.0, 7.1, 6.2]
	Fixed_wind_commend = ['2000']
	for i in range(len(Fixed_wind_list)):
		Fixed_wind_commend.append(Vwind_dict[Fixed_wind_list[i]])
	return Fixed_wind_commend

def CHANGE_WIND():
	Vary_wind_list = [8.0]
	for i in range(2*len(Vwind_map)):
		Vary_wind_list.append(random.choice(Vwind_map))
	return Vary_wind_list



