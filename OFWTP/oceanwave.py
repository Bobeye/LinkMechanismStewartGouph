import math
import numpy as np
import time

# Irregular ocean wave generation
# cylinder structure motion in waves

# Structure behaviour in ocean
mass = 10000.0			# mass of the structure /kg
am = 100.0			# hydrodynamics mass coefficient (Ns^2/m = kg)
bd = 0.04				# hydrodynamics damping coefficient (N/m = kg/s)
cs = 1000000.0			# restoring spring coefficient
G = 9.8					# Gravity
PI = math.pi 			# PI
PitchMax = 0.32 		# Maximum pitching range of the floating platform
# OCEAN WAVE SPECTRUM
## JONSWAP WAVE SPCTRUM
Fpeak = 3.3				# peakedness factor for JONSWARP wave spcetra
# Irregular ocean wave
w_sample = 0.02			# sample omega for inverse FFT




def STRUCTURE_IN_OCEAN(A0,w0,phi0):
	# Heave Amplitude
	A1 = A0 * math.exp(-w0*2*PI/G) * math.sqrt( (((cs-am*(w0**2))**2) + ((bd*w0)**2)) / (((cs-(mass+am)*(w0**2))**2)) + ((bd*w0)**2) )
	# Phase shift
	p_nume = - mass * bd * (w0)**3
	p_deno = (cs-am*(w0**2)) * ((cs-(mass+am)*(w0**2))) + ((bd*w0)**2)
	if p_nume == p_deno:
		phi1 = PI/2
	elif p_nume == -p_deno:
		phi1 = -PI/2
	else:
		phi1 = math.atan(p_nume/p_deno)
	return [A1, phi1]

def OCEAN_SPECTRUM(sigHeight,Tpeak,omega):
	OSw = omega
	OShsig = sigHeight
	OSTp = Tpeak
	OST = 2 * PI / OSw			# period
	OSwp = 2 * PI / OSTp		# peak omega
	# OSd for JONSWAP calculation, a step function related with omega
	if OSw <= OSwp:
		OSd = 0.07
	else:
		OSd = 0.09
	# A for JONSWAP calculation
	OSJA = math.exp(-((((OSw/OSwp)-1) / (OSd*1.414))**2))
	OS = (320 * (OShsig**2) / ((OSTp**4) * (OSw**5))) * math.exp((-1950 / ((OSTp**4)*(OSw**4)))) * (Fpeak**OSJA)
	return OS

def WIND_TO_WAVE(windspeed):
	WW_windspeed = windspeed
	WW_sig_height = 0.21 * (IW_windspeed**2) / G
	WW_peak_time = 2 * PI * IW_windspeed / (0.877 * G)
	return [WW_sig_height,WW_peak_time]

def IRREGULAR_WAVE(windspeed,currenttime):
	IW_windspeed = windspeed
	IW_t = currenttime
	IW_sig_height = 0.21 * (IW_windspeed**2) / G
	IW_peak_time = 2 * PI * IW_windspeed / (0.877 * G)
	IW_A = 0
	IW_alpha=0
	IW_Z=0
	IW_Zalpha=0
	for f in range(1000):
		f = f/100.0+0.01
		IWphi = np.random.uniform(-PI/2.0, PI/2.0, size=1)[0]		# random initial phase
		IW_Zphi = STRUCTURE_IN_OCEAN(IW_A,2*PI*f,IWphi)[1]
		IW_A = IW_A + math.sqrt(2*w_sample*OCEAN_SPECTRUM(IW_sig_height,IW_peak_time,2*PI*f)) * math.cos(2*PI*f*IW_t + IWphi)
		IW_alpha = IW_alpha + ((((2*PI*f)**2)/G) * IW_A)
		IW_Z = IW_Z + STRUCTURE_IN_OCEAN(IW_A,2*PI*f,IW_Zphi)[0] * math.cos(2*PI*f*IW_t + IW_Zphi)
		IW_Zalpha = IW_Zalpha + ((((2*PI*f)**2)/G) * IW_Z) * math.cos(2*PI*f*IW_t + IW_Zphi)
	if IW_alpha < 0:
		IW_alpha = -(IW_alpha % (PI/2.0))
	else:
		IW_alpha = IW_alpha % (PI/2.0)
	if IW_Zalpha < 0:
		IW_Zalpha = -(IW_Zalpha % (PI/2.0))
	else:
		IW_Zalpha = IW_Zalpha % (PI/2.0)
	return [IW_A,IW_alpha,IW_Z,IW_Zalpha]

# def main():
	

# if __name__=='__main__':
# 	main()