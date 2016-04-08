import math
import time
import random
import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import inv
import itertools


# Mechanical Parameters
BOTTOM_RADIUS = 119.3649864910141897009273117677145601037135856257366363864 # distance from the center of the bottom plate to the servo center
TOP_RADIUS = 74.33034373659252761306004106965698325724492756860430780281 # distance from the center of the top plate to the top joint
BOTTOM_ANGLE = 0.546166563433787740559629712911971244663191407124391241530
TOP_ANGLE = 0.343023940420703397073528599413809616687563147674740286598
LINKA = 75.22 # length of the body link connected to the servo, the first part of the link-mechanism leg
LINKB = 120.00# length of the body link connected to the top plate, the second part of the link-mechanism leg
ZEROHEIGHT = 200.0

SERVOHEIGHT = 41.5	
SERVOANGLE_MIN = 0 # zero position is corresponding to the z axis in right-handed coordinates system, clockwise moving is considered as above zero
SERVOANGLE_MAX = 90
SERVOSPEED_MAX = 400
SERVOSPEED_MIN = 0

# WORKINGRANGE = [15.0,15.0,15.0,0.15,0.15,0.15]





# COORDINATS & NUMBER TAG:
# The space origin is always following the right-handed coordinats system. The origin is located at the center of the bottom plate.
# Num 0 tag is always referring to the servo located within the third angle projection. The tagging sequence is following the direcion of anti-clockwise,
# which means the tag 1 is reffering to the servo locating on the right side of the servo 0.
class CONFIGURE:
	# data check
	def PositiveDataCheck(self):
		if BOTTOM_RADIUS <= 0 or TOP_RADIUS <= 0 or BOTTOM_ANGLE <= 0 or TOP_ANGLE <= 0 or LINKA <= 0 or LINKB <= 0:
			print("Warning! Strcture dimensions must be positive!")


	def OriginPosition(self):
		BottomCoordinates = [[BOTTOM_RADIUS * math.cos(BOTTOM_ANGLE), -BOTTOM_RADIUS * math.sin(BOTTOM_ANGLE), 0],
						 [BOTTOM_RADIUS * math.cos(BOTTOM_ANGLE), BOTTOM_RADIUS * math.sin(BOTTOM_ANGLE), 0],
						 [-BOTTOM_RADIUS * math.sin(math.radians(30)-BOTTOM_ANGLE), BOTTOM_RADIUS * math.cos(math.radians(30)-BOTTOM_ANGLE), 0],
						 [-BOTTOM_RADIUS * math.sin(math.radians(30)+BOTTOM_ANGLE), BOTTOM_RADIUS * math.cos(math.radians(30)+BOTTOM_ANGLE), 0],
						 [-BOTTOM_RADIUS * math.sin(math.radians(30)+BOTTOM_ANGLE), -BOTTOM_RADIUS * math.cos(math.radians(30)+BOTTOM_ANGLE), 0],
						 [-BOTTOM_RADIUS * math.sin(math.radians(30)-BOTTOM_ANGLE), -BOTTOM_RADIUS * math.cos(math.radians(30)-BOTTOM_ANGLE), 0]]
		# print('BottomCoordinates = ',BottomCoordinates)
		TopCoordinates = [[TOP_RADIUS * math.cos(TOP_ANGLE), -TOP_RADIUS * math.sin(TOP_ANGLE), ZEROHEIGHT],
						 [TOP_RADIUS * math.cos(TOP_ANGLE), TOP_RADIUS * math.sin(TOP_ANGLE), ZEROHEIGHT],
						 [-TOP_RADIUS * math.sin(math.radians(30)-TOP_ANGLE), TOP_RADIUS * math.cos(math.radians(30)-TOP_ANGLE), ZEROHEIGHT],
						 [-TOP_RADIUS * math.sin(math.radians(30)+TOP_ANGLE), TOP_RADIUS * math.cos(math.radians(30)+TOP_ANGLE), ZEROHEIGHT],
						 [-TOP_RADIUS * math.sin(math.radians(30)+TOP_ANGLE), -TOP_RADIUS * math.cos(math.radians(30)+TOP_ANGLE), ZEROHEIGHT],
						 [-TOP_RADIUS * math.sin(math.radians(30)-TOP_ANGLE), -TOP_RADIUS * math.cos(math.radians(30)-TOP_ANGLE), ZEROHEIGHT]]
		# print('TopCoordinates = ',TopCoordinates)
		ServoCoordinates = BottomCoordinates
		for i in range(6):
			ServoCoordinates[i][2] = SERVOHEIGHT
		# print('ServoCoordinates',ServoCoordinates)
		InitialCoordinates = [BottomCoordinates, TopCoordinates, ServoCoordinates]
		return InitialCoordinates


	def TopplateMotion(self, TopCoordinates, TopMotion):
		TempTop = TopCoordinates
		temptopz = TempTop[0][2]
		for i in range(6):
			TempTop[i][2] = 0.0
		Top = TempTop
		deltaX = TopMotion[0]
		deltaY = TopMotion[1]
		deltaZ = TopMotion[2]
		alpha = TopMotion[3]
		belta = TopMotion[4]
		gamma = TopMotion[5]
		def S(angle):
			return math.sin(angle)
		def C(angle):
			return math.cos(angle)
		RotationM = [[C(gamma) * C(belta) , -S(gamma) * C(alpha) + C(gamma) * S(belta) * S(alpha) , S(gamma) * S(alpha) + C(gamma) * S(belta) * C(alpha)],
						  [S(gamma) * C(belta) , C(gamma) * C(alpha) + S(gamma) * S(belta) * S(alpha) , -C(gamma) * S(alpha) + S(gamma) * S(belta) * C(alpha)],
						  [-S(belta) , C(belta) * S(alpha) , C(belta) * C(alpha)]]
		TranslationM = [deltaX , deltaY, deltaZ]
		for i in range(6):
			for j in range(3):
				Top[i][j] = RotationM[j][0] * TempTop[i][0] + RotationM[j][1] * TempTop[i][1] + RotationM[j][2] * TempTop[i][2] + TranslationM[j]
			Top[i][2] = Top[i][2] + temptopz
		# print('After-Motion Top plate Coordinates', Top)
		return Top

	def LegLength(self, AimTopplate, ServoCoordinates):
		# Calculate leg length
		LegLength = [0.0,1.0,2.0,3.0,4.0,5.0]
		for i in range(6):
			TempDistance = 0.0
			for j in range(3):
				TempDistance = TempDistance + ((AimTopplate[i][j]-ServoCoordinates[i][j])**2)
			LegLength[i] = math.sqrt(TempDistance)
		# print('Leglength = ', LegLength)
		return LegLength

	def InverseKinematics(self, AimTopplate, ServoCoordinates, LinkA, LinkB):
		# Calculate leg length
		LegLength = [0.0,1.0,2.0,3.0,4.0,5.0]
		for i in range(6):
			TempDistance = 0.0
			for j in range(3):
				TempDistance = TempDistance + ((AimTopplate[i][j]-ServoCoordinates[i][j])**2)
			LegLength[i] = math.sqrt(TempDistance)
		# print('Leglength = ', LegLength)
		# Calculate leg direction
		LegAngle = AimTopplate
		TempLegAngle = AimTopplate
		for i in range(6):
			for j in range(3):
				LegAngle[i][j] = AimTopplate[i][j] - ServoCoordinates[i][j]
				TempLegAngle[i][j] = LegAngle[i][j]
			# LegAngle[i][0], LegAngle[i][1] = LegAngle[i][1], -LegAngle[i][0] # Switch the coordinates system from the right-handed to a standard 2D coordinates
		# print('LegAngle', LegAngle)
		YT = range(6)
		ZT = range(6)
		for i in range(6):
			ZT[i] = LegAngle[i][2]
			if i <= 1:
				YT[i] = LegAngle[i][1]
			elif i == 2:
				axisrot = math.pi*2/3
				ca = math.cos(axisrot)
				sa = math.sin(axisrot)
				x0 = LegAngle[i][0]
				y0 = LegAngle[i][1]
				YT[i] = y0 * ca - x0 * sa
			elif i == 3:
				axisrot = math.pi*2/3
				ca = math.cos(axisrot)
				sa = math.sin(axisrot)
				x0 = LegAngle[i][0]
				y0 = LegAngle[i][1]
				YT[i] = y0 * ca - x0 * sa
			elif i == 4:
				axisrot = -math.pi*2/3
				ca = math.cos(axisrot)
				sa = math.sin(axisrot)
				x0 = LegAngle[i][0]
				y0 = LegAngle[i][1]
				YT[i] = y0 * ca - x0 * sa
			elif i == 5:
				axisrot = -math.pi*2/3
				ca = math.cos(axisrot)
				sa = math.sin(axisrot)
				x0 = LegAngle[i][0]
				y0 = LegAngle[i][1]
				YT[i] = y0 * ca - x0 * sa
		# print('YT', YT)
		# print('ZT', ZT)

		ALPHA = [0.0,1.0,2.0,3.0,4.0,5.0]
		AimServoAngle = [0.0,1.0,2.0,3.0,4.0,5.0]


		# Motion Planning
		for i in range(6):
			M = ((LegLength[i] ** 2) + (LinkA ** 2) - (LinkB ** 2)) / (2 * LinkA * ZT[i])
			N = YT[i] / ZT[i]
			# print('M', M)
			# print('N', N)
			# cos(alpha) has two results
			alpha = 0
			if i % 2 == 1:
				Alphaa = (M * N + (math.sqrt((N**2) - (M**2) + 1.0))) / (N**2 + 1.0)
				Alphab = (M * N - (math.sqrt((N**2) - (M**2) + 1.0))) / (N**2 + 1.0)
				alphaa = math.acos(Alphaa)
				alphab = math.acos(Alphab)
				# print('a', alphaa)
				# print('b', alphab)
				if abs(alphaa) <= 1.5708:
					alpha = alphaa
				elif abs(alphab) <= 1.5708:
					alpha = alphab
				ALPHA[i] = alpha
				AimServoAngle[i] = 90 - math.degrees(ALPHA[i])
			else:
				Alphaa = (-(M * N) + (math.sqrt((N**2) - (M**2) + 1.0))) / (N**2 + 1.0)
				Alphab = (-(M * N) - (math.sqrt((N**2) - (M**2) + 1.0))) / (N**2 + 1.0)
				alphaa = math.acos(Alphaa)
				alphab = math.acos(Alphab)
				# print('a', alphaa)
				# print('b', alphab)
				if abs(alphaa) <= 1.5708:
					alpha = alphaa
				elif abs(alphab) <= 1.5708:
					alpha = alphab
				ALPHA[i] = alpha
				AimServoAngle[i] = 90 - math.degrees(ALPHA[i])
		# print('ALPHA', ALPHA)
		# print('AimServoAngle = ', AimServoAngle)
		return AimServoAngle

###################################################################################




def main():
	# Motor configuration
	ports = pypot.dynamixel.get_available_ports()
	if not ports:
		raise IOError('no port found!')
	print('ports found', ports)
	print('connecting on the first available port:', ports[0])
	dxl_io = pypot.dynamixel.DxlIO(ports[0])
	Motors = dxl_io.scan(range(10))
	print('Motors ID', Motors)
	##########Speed Set#####################
	for i in Motors:
		dxl_io.set_moving_speed({i: 1000})
	# intial motor position
	LastMotorPos = [0,0,0,0,0,0]
	# zero all the motors
	def ZEROSERVO():
		for i in Motors:
			dxl_io.set_goal_position({i: 0})
			time.sleep(0.01)
		LastMotorPos = [0,0,0,0,0,0]
		print('Zero all the motors...')
	# classify motors
	leftMotors = [0]
	rightMotors = [0]
	for i in Motors:
		if i % 2 == 0:
			leftMotors.append(i)
		else:
			rightMotors.append(i)
	leftMotors.pop(0)
	rightMotors.pop(0)
	print('left set of motors:', leftMotors)
	print('right set of motors:', rightMotors)
	# Move the servos to desired angle position
	def SERVOMOVE(MotorPos,Period):
		permit = 0
		ServoSpeed = range(6)
		MaxSpeed = 360 # the max servo speed is 360 degree/s, coresponding to 360
		SafeSpeedFac = 1.5 # 1.5 is the safe factor for servo speed
		# Speed Settings
		# get current servo position
		CurrentPos = range(6)
		PosDiff = range(6)
		# CurrentInfo = dxl_io.get_present_position_speed_load(Motors)
		for i in range(6):
			CurrentPos[i] = LastMotorPos[i]
			PosDiff[i] = abs(MotorPos[i] - abs(CurrentPos[i]))
			ServoSpeed[i] = PosDiff[i] / Period
			ServoSpeed[i] = ServoSpeed[i]
			if ServoSpeed[i] > SafeSpeedFac * MaxSpeed:
				permit = 1
			if ServoSpeed[i] > MaxSpeed:
				ServoSpeed[i] = MaxSpeed
		for i in Motors:
			dxl_io.set_moving_speed({i: ServoSpeed[i-1]})
		# print('Current sPEED:', ServoSpeed)
		# print(CurrentPos)
		for i in MotorPos:
			if abs(i) > 45:
				permit = 2
		if permit == 1:
			print('WARNING!!!!!!! MOTOR SPEED OVER RANGE!!!')
		if permit == 2:
			# print('MOTOR POSITION:', MotorPos)
			print('WARNING!!!!!!! MOTOR OVER RANGE!!!')
		else:
			print('MOTOR POSITION:', MotorPos)
			dxl_io.set_goal_position({1: -MotorPos[1]})
			dxl_io.set_goal_position({2: MotorPos[2]})
			dxl_io.set_goal_position({3: -MotorPos[3]})
			dxl_io.set_goal_position({4: MotorPos[4]})
			dxl_io.set_goal_position({5: -MotorPos[5]})
			dxl_io.set_goal_position({6: MotorPos[0]})

	# StewartControl
	def STEWARTMOVE(MT):
		# initial the configure class
		configure = CONFIGURE()
		# Initial coordinates setup
		InitialCordinates=configure.OriginPosition()
		BottomCoordinates=InitialCordinates[0]
		TopCoordinates=InitialCordinates[1]
		ServoCoordinates=InitialCordinates[2]
		AimTopplate = configure.TopplateMotion(TopCoordinates, MT)
		AimServoPos = configure.InverseKinematics(AimTopplate, ServoCoordinates, LINKA, LINKB)
		print(AimServoPos) # in degrees
		return AimServoPos

	# Move Stewart
	Mtop = [0.0,0.0,0.0,0.0,0.0,-0.45]
	ServoAngle = STEWARTMOVE(Mtop)
	SERVOMOVE(ServoAngle,0.01)
	LastMotorPos = ServoAngle
	time.sleep(2)


main()