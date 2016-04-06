import math
import structure as stru

def IFFEASIBLE():
	BR = stru.BOTTOM_RADIUS
	TR = stru.TOP_RADIUS
	BA = stru.BOTTOM_ANGLE
	TA = stru.TOP_ANGLE
	LA = stru.LINKA
	LB = stru.LINKB
	H0 = stru.ZEROHEIGHT
	HS = stru.SERVOHEIGHT

	if BR <= 0 or TR <= 0 or BA <= 0 or TA <= 0 or LA <= 0 or LB <= 0:
		print("ERROR")
		print("Structure parameters must be positive")
		return 1

	# Architecture Singularity
	if BA >= math.radians(30)*0.98 or TA >= math.radians(30)*0.98 or BA <= math.radians(30)*1.02 or TA <= math.radians(30)*1.02:
		print("ERROR")
		print("Archetecture Singularity")
		return 1

	
	print("Feasible structure checked")
	return 0