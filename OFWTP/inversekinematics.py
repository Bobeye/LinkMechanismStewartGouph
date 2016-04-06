import configure as CO 



def INVERSE_KINEMATICS(TopPlateMotion):
	configure = CO.CONFIGURE()
	InitialCordinates=configure.OriginPosition()
	CO_BottomC=InitialCordinates[0]
	CO_TopC=InitialCordinates[1]
	CO_ServoC=InitialCordinates[2]
	TopMotion = TopPlateMotion # Angle in radius, given desired topplate motion
	AimTopplate = configure.TopplateMotion(CO_TopC, TopMotion)
	AimServoPos = configure.InverseKinematics(AimTopplate, CO_ServoC, CO.LINKA, CO.LINKB)
	return AimServoPos


if __name__=='__main__':
    print(INVERSE_KINEMATICS([0.0,0.0,0.0,0.0,0.0,0.0]))