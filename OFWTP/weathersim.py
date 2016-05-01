import windtunnel as WT
import oceanwave as OC
import windvelocity as WV
import inversekinematics as IK
# import movestewart as MS
import matplotlib.pyplot as plt
import math
import numpy as np
import time


# wind speed map setup and corresponding ocean wave spectrum
windmap = WV.Vwind_map
sorted_windmap = sorted(windmap)
sig_sorted_windmap = [0]
for wind_speed in sorted_windmap:
	if wind_speed not in sig_sorted_windmap:
		sig_sorted_windmap.append(wind_speed)
sig_sorted_windmap.remove(0)
print("signature wind map: ", sig_sorted_windmap)
num_wind_speed = 0
ocean_spectrum = [[0]]
for wind_speed in sig_sorted_windmap:
	sig_height = OC.WIND_TO_WAVE(wind_speed)[0]
	peak_time = OC.WIND_TO_WAVE(wind_speed)[1]
	for i in np.linspace(0.01,5.00,num=1000):
		ocean_spectrum[num_wind_speed].append(OC.OCEAN_SPECTRUM(sig_height,peak_time,i))
	num_wind_speed = num_wind_speed + 1
	ocean_spectrum.append([0])
ocean_spectrum.pop
# Plot the ocean wave spectrum
plt.figure(1)
for i in range(len(sig_sorted_windmap)):
	plt.plot(np.linspace(0.01,5.00,num=1001), ocean_spectrum[i])
plt.legend(sig_sorted_windmap, loc='upper left')
plt.xlabel('ocean wave frequency/Hz')
plt.ylabel('wave energy')
plt.title('JONSWAP Ocean Wave Spectrum \n for OFWTP Simulation \n')



print(WV.FIXED_CHANGE_WIND())

# OceanWave Generation
for wind_speed in sig_sorted_windmap:
	start = time.time()
	current = time.time()
	simulate_data_list = [[0,0]]
	while current-start <= 1:
		simulate_wave_data = OC.IRREGULAR_WAVE(wind_speed,current)
		simulate_height = simulate_wave_data[2]
		simulate_slope = simulate_wave_data[3] / 6.0
		# simulate_slope = simulate_wave_data[3]
		if simulate_slope >= 0.25:
			simulate_slope = 0.25
		elif simulate_slope <= -0.25:
			simulate_slope = -0.25
		simulate_data_list.append([simulate_height,simulate_slope])
		current = time.time()
simulate_height_list = [0]
simulate_slope_list = [0]
for i in range(len(simulate_data_list)):
	simulate_height_list.append(simulate_data_list[i][0])
	simulate_slope_list.append(simulate_data_list[i][1])
plt.figure(2)
plt.subplot(211)
plt.plot(simulate_height_list, color='red')
plt.xlabel('time/second')
plt.ylabel('simulated wave height/m')
plt.title('Simulated Ocean Wave Data (Height) \n')
plt.subplot(212)
plt.plot(simulate_slope_list, color='green')
plt.xlabel('time/second')
plt.ylabel('simulated wave slope/rad')
plt.title('Simulated Ocean Wave Data (Slope) \n')
plt.show()