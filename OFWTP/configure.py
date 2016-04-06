import math
import time
import random
import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import inv


# Mechanical Parameters
BOTTOM_RADIUS = 119.3649864910141897009273117677145601037135856257366363864 # distance from the center of the bottom plate to the servo center
TOP_RADIUS = 74.33034373659252761306004106965698325724492756860430780281 # distance from the center of the top plate to the top joint
BOTTOM_ANGLE = 0.546166563433787740559629712911971244663191407124391241530
TOP_ANGLE = 0.343023940420703397073528599413809616687563147674740286598
LINKA = 75.22 # length of the body link connected to the servo, the first part of the link-mechanism leg
LINKB = 120.00# length of the body link connected to the top plate, the second part of the link-mechanism leg
ZEROHEIGHT = 200.0
SERVOHEIGHT = 41.5


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

	def MonteCarlo(self):
		sampleResolution = 12.0
		sampleStep = 4.0
		sampleNum = int((sampleResolution*2+1)/sampleStep)**6
		# Error range set
		# deltaTopplate = [0.1,0.1,0.1,0.1,0.1,0.1] # angls are in degree!!!!!
		# deltaTopplate = [1.0,0.0,0.0,0.0,0.0,0.0] # angls are in degree!!!!!
		# deltaTopplate = [0.0,1.0,0.0,0.0,0.0,0.0] # angls are in degree!!!!!
		# deltaTopplate = [0.0,0.0,1.0,0.0,0.0,0.0] # angls are in degree!!!!!
		deltaTopplate = [0.19,0.28,0.29,0.063,0.063,0.2] # angls are in degree!!!!!
		# Random
		sampleList = [[0],[0],[0],[0],[0],[0]]
		sampleTopplate = [0,0,0,0,0,0]
		tempsampleList = [0]
		for i in range(6):
			tempsampleList = np.random.uniform(-deltaTopplate[i],deltaTopplate[i],sampleNum)
			for j in range(sampleNum):
				sampleList[i].append(tempsampleList[j])
			sampleList[i].pop
		for i in [3,4,5]:
			for j in range(len(sampleList[i])):
				sampleList[i][j] = math.radians(sampleList[i][j])
		# print('sampleList',sampleList)
		print('MonteCarlo sampleNum:', sampleNum)
		return sampleList

	def ForwardKinematics(sefl, ServoAngle, ServoCoordinates, TopCoordinates, ZeroTopplate, LinkA, LinkB, DBP):
		# Degree to radius
		for i in range(6):
			ServoAngle[i] = math.radians(ServoAngle[i])
		# Define the position of the universal joint between LINKA and LINKB
		UniversalJointAB = ServoCoordinates
		UniversalJointAB = [ [ServoCoordinates[0][0] , ServoCoordinates[0][1]-(LINKA*math.sin(ServoAngle[0])) , ServoCoordinates[0][2]+(LINKA*math.cos(ServoAngle[0]))],
							 [ServoCoordinates[1][0] , ServoCoordinates[1][1]+(LINKA*math.sin(ServoAngle[1])) , ServoCoordinates[1][2]+(LINKA*math.cos(ServoAngle[1]))],
							 [ServoCoordinates[2][0]+(LINKA*math.sin(ServoAngle[2])*math.cos(BOTTOM_ANGLE)) , ServoCoordinates[2][1]+(LINKA*math.sin(ServoAngle[2])*math.sin(BOTTOM_ANGLE)) , ServoCoordinates[2][2]+(LINKA*math.cos(ServoAngle[2]))],
							 [ServoCoordinates[3][0]-(LINKA*math.sin(ServoAngle[3])*math.cos(BOTTOM_ANGLE)) , ServoCoordinates[3][1]-(LINKA*math.sin(ServoAngle[3])*math.sin(BOTTOM_ANGLE)) , ServoCoordinates[3][2]+(LINKA*math.cos(ServoAngle[3]))],
							 [ServoCoordinates[4][0]-(LINKA*math.sin(ServoAngle[4])*math.cos(BOTTOM_ANGLE)) , ServoCoordinates[4][1]+(LINKA*math.sin(ServoAngle[4])*math.sin(BOTTOM_ANGLE)) , ServoCoordinates[4][2]+(LINKA*math.cos(ServoAngle[4]))],
							 [ServoCoordinates[5][0]+(LINKA*math.sin(ServoAngle[5])*math.cos(BOTTOM_ANGLE)) , ServoCoordinates[5][1]-(LINKA*math.sin(ServoAngle[5])*math.sin(BOTTOM_ANGLE)) , ServoCoordinates[5][2]+(LINKA*math.cos(ServoAngle[5]))]]
		# print('UniversalJointAB:', UniversalJointAB)
		# Check LINKA's working range
		def CrossProduct(V1,V2): # cross product of two vectors
			for i in range(3):
				crossproduct = [V1[1]*V2[2]-V2[1]*V1[2],V1[0]*V2[2]-V2[0]*V1[2],V1[0]*V2[1]-V1[1]*V2[0]]
			return crossproduct
		def CCW(A,B,C): # See if three points are listed counter clock wise
			SegAB = [0,0,0]
			SegAC = [0,0,0]
			for i in range(3):
				SegAB[i] = B[i] - A[i]
				SegAC[i] = C[i] - A[i]
			if CrossProduct(SegAB,SegAC)[2] > 0:
				return True
			else:
				return False
		def Intersect(PA1,PA2,PB1,PB2): # See if line segment PA1-PA2 and PB1-PB2 interacts, TRUE for intersect
			return CCW(PA1,PB1,PB2) != CCW(PA2,PB1,PB2) and CCW(PA1,PA2,PB1) != CCW(PA1,PA2,PB2)
		def Coplanar(A,B,C,D): # See if four points are coplanar
			SegAB = [0,0,0]
			SegAC = [0,0,0]
			SegAD = [0,0,0]
			for i in range(3):
				SegAB[i] = B[i] - A[i]
				SegAC[i] = C[i] - A[i]
				SegAD[i] = D[i] - A[i]
			coplanarVec = CrossProduct(CrossProduct(SegAB,SegAC),CrossProduct(SegAB,SegAD))
			if coplanarVec[0] == 0 and coplanarVec[1] == 0 and coplanarVec[2] == 0:
				return True
			else:
				return False
		# first, see if the segment points of the two links are coplanar, second, see if the two links are interacting
		for i in range(6):
			if i < 5:
				if Coplanar(ServoCoordinates[i],UniversalJointAB[i],ServoCoordinates[i+1],UniversalJointAB[i+1]) == True:
					if Intersect(ServoCoordinates[i],UniversalJointAB[i],ServoCoordinates[i+1],UniversalJointAB[i+1]) == True:
						print("Warning! Links have intersetions!!!")
					else:
						print("Links are safe to go!")
			else:
				if Coplanar(ServoCoordinates[5],UniversalJointAB[5],ServoCoordinates[0],UniversalJointAB[0]) == True:
					if Intersect(ServoCoordinates[5],UniversalJointAB[5],ServoCoordinates[0],UniversalJointAB[0]) == True:
						print("Warning! Links have intersetions!!!")
					else:
						print("Links are safe to go!")	

		# Newton-Raphson Method
		print('Newton-Raphson is on!!!')
		print('Initial Top Plate = ', TopCoordinates)
		print('Initial Servo Plate = ', ServoCoordinates)
		print('Given servo angle = ', ServoAngle)
		print('UniversalJointAB pos = ', UniversalJointAB)

		def F(TopCoordinates,TopMotion,UniversalJointAB,LinkB):
			F = [0.00000000,0.000000000,0.0000000000,0.00000000000,0.0000000000,0.0000000000]
			TempTop = TopCoordinates
			Top = TopCoordinates
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
					Top[i][j] = RotationM[j][0] * TempTop[i][0] + RotationM[j][1] * TempTop[i][1] + RotationM[j][2] * TempTop[i][2] + TranslationM[j] - UniversalJointAB[i][j]
				F[i] = math.sqrt(Top[i][0] ** 2 + Top[i][1] ** 2 + Top[i][2] ** 2) - LinkB
			return F
		# TopMotion = [0.0,0.0,0.0,0.0,0.0,0.0] # Angle in radius
		# F = F(TopCoordinates,TopMotion,UniversalJointAB,LinkB)
		# print('text F result', F)

		def f(TopCoordinates,TopMotion,UniversalJointAB,LinkB):
			TempTop = TopCoordinates
			Top = TopCoordinates
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
					Top[i][j] = RotationM[j][0] * TempTop[i][0] + RotationM[j][1] * TempTop[i][1] + RotationM[j][2] * TempTop[i][2] + TranslationM[j] - UniversalJointAB[i][j]
			f = Top
			return f

		def dF(TopCoordinates,TopMotion):
			dF = [[[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]]],
				  [[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]]],
				  [[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]]],
				  [[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]]],
				  [[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]]],
				  [[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]]],]
			Top = TopCoordinates
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
			for i in range(6):
				# d(f)/d(deltaX) Y Z
				dF[i][0] = [1.0,0.0,0.0]
				dF[i][1] = [0.0,1.0,0.0]
				dF[i][2] = [0.0,0.0,1.0]
				# d(f)/d(alpha)
				dF[i][3] = [S(gamma)*S(alpha)*Top[i][1] + C(gamma)*S(belta)*C(alpha)*Top[i][1] + S(gamma)*C(alpha)*Top[i][2] - C(gamma)*S(belta)*S(alpha)*Top[i][2],
							-C(gamma)*S(alpha)*Top[i][1] + S(gamma)*S(belta)*C(alpha)*Top[i][1] - C(gamma)*C(alpha)*Top[i][2] - S(gamma)*S(belta)*S(alpha)*Top[i][2],
							C(belta)*C(alpha)*Top[i][1] - C(belta)*S(alpha)*Top[i][2]]
				# d(f)/d(belta)
				dF[i][4] = [-C(gamma)*S(belta)*Top[i][0] + C(gamma)*C(belta)*S(alpha)*Top[i][1] + C(gamma)*C(belta)*C(alpha)*Top[i][2],
							-S(gamma)*S(belta)*Top[i][0] + S(gamma)*C(belta)*S(alpha)*Top[i][1] + S(gamma)*C(belta)*C(alpha)*Top[i][2],
							-C(belta)*Top[i][0] - S(belta)*S(alpha)*Top[i][1] - S(belta)*C(alpha)*Top[i][2]]
				# d(f)/d(gamma)
				dF[i][5] = [-S(gamma)*C(belta)*Top[i][0] - C(gamma)*C(alpha)*Top[i][1] - S(gamma)*S(belta)*S(alpha)*Top[i][1] + C(gamma)*S(alpha)*Top[i][2] - S(gamma)*S(belta)*C(alpha)*Top[i][2],
							C(gamma)*C(belta)*Top[i][0] - S(gamma)*C(alpha)*Top[i][1] + C(gamma)*S(belta)*S(alpha)*Top[i][1] + S(gamma)*S(alpha)*Top[i][2] + C(gamma)*S(belta)*C(alpha)*Top[i][2],
							0]
			return dF
		# TopMotion = [0.0,0.0,0.0,0.0,0.0,0.0] # Angle in radius
		# dF = dF(TopCoordinates,TopMotion)
		# print('text dF result', dF)

		# NewtonRaphson: # Xn+1 = Xn - f(Xn)/df(Xn)
		resolution = 0.1
		count = 1
		start = time.time()
		CurrentTopMotion = [0.0,0.0,0.0,0.0,0.0,0.0]
		NextTopMotion = [0.0,0.0,0.0,0.0,0.0,0.0]
		TopCoordinates = [[70.0000000000026, -24.999999999992898, 208.79999999999063], [69.9999999999976, 25.000000000007095, 208.79999999999563], [-13.349364905396241, 73.12177826490947, 208.80000000000877], [-56.65063509461567, 48.12177826490514, 208.80000000001058], [-56.65063509460605, -48.121778264916266, 208.80000000000098], [-13.349364905381618, -73.12177826491194, 208.79999999999416]]
		F0 = F(TopCoordinates,CurrentTopMotion,UniversalJointAB,LinkB)
		TopCoordinates = [[70.0000000000026, -24.999999999992898, 208.79999999999063], [69.9999999999976, 25.000000000007095, 208.79999999999563], [-13.349364905396241, 73.12177826490947, 208.80000000000877], [-56.65063509461567, 48.12177826490514, 208.80000000001058], [-56.65063509460605, -48.121778264916266, 208.80000000000098], [-13.349364905381618, -73.12177826491194, 208.79999999999416]]
		dF0 = dF(TopCoordinates,CurrentTopMotion)
		TopCoordinates = [[70.0000000000026, -24.999999999992898, 208.79999999999063], [69.9999999999976, 25.000000000007095, 208.79999999999563], [-13.349364905396241, 73.12177826490947, 208.80000000000877], [-56.65063509461567, 48.12177826490514, 208.80000000001058], [-56.65063509460605, -48.121778264916266, 208.80000000000098], [-13.349364905381618, -73.12177826491194, 208.79999999999416]]
		f0 = f(TopCoordinates,CurrentTopMotion,UniversalJointAB,LinkB)
		TopCoordinates = [[70.0000000000026, -24.999999999992898, 208.79999999999063], [69.9999999999976, 25.000000000007095, 208.79999999999563], [-13.349364905396241, 73.12177826490947, 208.80000000000877], [-56.65063509461567, 48.12177826490514, 208.80000000001058], [-56.65063509460605, -48.121778264916266, 208.80000000000098], [-13.349364905381618, -73.12177826491194, 208.79999999999416]]
		for i in range(6): # [deltaX, deltaY, deltaZ, alpha, belta, gamma]
			Sum = 0.0
			for j in range(6): # leg 0 ,1 ,2 3 4 5
				Sum = Sum + ( F0[j] / (2 * (dF0[j][i][0] * f0[j][0] + dF0[j][i][1] * f0[j][1] + dF0[j][i][2] * f0[j][2])) )
			NextTopMotion[i] = CurrentTopMotion[i] - Sum
		print ('NextTopMotion = ', NextTopMotion)
		print ('TP', TopCoordinates)
		F1 = F(TopCoordinates,NextTopMotion,UniversalJointAB,LinkB)
		print('PreviousF: ', F0)
		print('NextF: ', F1)
		# Permit = 0
		# for i in range(6):
		# 	if abs(F1[i]) <= resolution:
		# 		Permit = Permit + 1
		# while Permit < 6:
		Sum = 0.0
		for i in range(6):
			Sum = Sum + F1[i]
		while Sum >= resolution:
			count = count + 1
			CurrentTopMotion = NextTopMotion
			TopCoordinates = [[70.0000000000026, -24.999999999992898, 208.79999999999063], [69.9999999999976, 25.000000000007095, 208.79999999999563], [-13.349364905396241, 73.12177826490947, 208.80000000000877], [-56.65063509461567, 48.12177826490514, 208.80000000001058], [-56.65063509460605, -48.121778264916266, 208.80000000000098], [-13.349364905381618, -73.12177826491194, 208.79999999999416]]
			F0 = F(TopCoordinates,CurrentTopMotion,UniversalJointAB,LinkB)
			TopCoordinates = [[70.0000000000026, -24.999999999992898, 208.79999999999063], [69.9999999999976, 25.000000000007095, 208.79999999999563], [-13.349364905396241, 73.12177826490947, 208.80000000000877], [-56.65063509461567, 48.12177826490514, 208.80000000001058], [-56.65063509460605, -48.121778264916266, 208.80000000000098], [-13.349364905381618, -73.12177826491194, 208.79999999999416]]
			dF0 = dF(TopCoordinates,CurrentTopMotion)
			TopCoordinates = [[70.0000000000026, -24.999999999992898, 208.79999999999063], [69.9999999999976, 25.000000000007095, 208.79999999999563], [-13.349364905396241, 73.12177826490947, 208.80000000000877], [-56.65063509461567, 48.12177826490514, 208.80000000001058], [-56.65063509460605, -48.121778264916266, 208.80000000000098], [-13.349364905381618, -73.12177826491194, 208.79999999999416]]
			f0 = f(TopCoordinates,CurrentTopMotion,UniversalJointAB,LinkB)
			TopCoordinates = [[70.0000000000026, -24.999999999992898, 208.79999999999063], [69.9999999999976, 25.000000000007095, 208.79999999999563], [-13.349364905396241, 73.12177826490947, 208.80000000000877], [-56.65063509461567, 48.12177826490514, 208.80000000001058], [-56.65063509460605, -48.121778264916266, 208.80000000000098], [-13.349364905381618, -73.12177826491194, 208.79999999999416]]
			for i in range(6): # [deltaX, deltaY, deltaZ, alpha, belta, gamma]
				Sum = 0.0
				for j in range(6): # leg 0 ,1 ,2 3 4 5
					Sum = Sum + ( F0[j] / (2 * (dF0[j][i][0] * f0[j][0] + dF0[j][i][1] * f0[j][1] + dF0[j][i][2] * f0[j][2])) )
				NextTopMotion[i] = CurrentTopMotion[i] - Sum
			print ('NextTopMotion = ', NextTopMotion)
			print ('TP', TopCoordinates)
			F1 = F(TopCoordinates,NextTopMotion,UniversalJointAB,LinkB)
			print('PreviousF: ', F0)
			print('NextF: ', F1)
			Sum = 0.0
			for i in range(6):
				Sum = Sum + F1[i]
			# Permit = 0
			# for i in range(6):
			# 	if F1[i] <= resolution:
			# 		Permit = Permit + 1
		end = time.time()
		print ('Iteration Period: ', count, 'Total Time', end-start)
		print ('Aim Topplate Motion: ', NextTopMotion)

def main():
# 1
	# initial the configure class
	configure = CONFIGURE()

# 2
	# Initial coordinates setup
	InitialCordinates=configure.OriginPosition()
	BottomCoordinates=InitialCordinates[0]
	TopCoordinates=InitialCordinates[1]
	ServoCoordinates=InitialCordinates[2]

# # 3
# 	# # Move the TOP PLATE
# 	# TopMotion = [0.0,0.0,0.0,0.0,0.0,0.0] # Angle in radius
# 	# AimTopplate = configure.TopplateMotion(TopCoordinates, TopMotion)

# # 4
# 	# Inverse Kinematics
# 	InitialCordinates=configure.OriginPosition()
# 	BottomCoordinates=InitialCordinates[0]
# 	TopCoordinates=InitialCordinates[1]
# 	ServoCoordinates=InitialCordinates[2]
# 	TopMotion = [0.0,0.0,0.0,0.0,0.0,-0.36] # Angle in radius, given desired topplate motion
# 	AimTopplate = configure.TopplateMotion(TopCoordinates, TopMotion)
# 	AimServoPos = configure.InverseKinematics(AimTopplate, ServoCoordinates, LINKA, LINKB)
# 	print(AimServoPos) # in degrees

# # 5
# 	# MonteCarlo Accuracy Analysis
# 	# Move top to zero

# 	# ZeroTopMotion = [0.1,0.1,0.1,0.0,0.0,0.0] # Angle in radius
# 	# ZeroAimTopplate = configure.TopplateMotion(TopCoordinates, ZeroTopMotion)
# 	# ZeroAimServoPos = configure.InverseKinematics(ZeroAimTopplate, ServoCoordinates, LINKA, LINKB)	

# 	InitialCordinates=configure.OriginPosition()
# 	BottomCoordinates=InitialCordinates[0]
# 	TopCoordinates=InitialCordinates[1]
# 	ServoCoordinates=InitialCordinates[2]

# 	print('top',TopCoordinates)


# 	ZeroTopMotion = [0.1,0.0,0.0,0.0,0.0,0.0] # Angle in radius
# 	ZeroAimTopplate = configure.TopplateMotion(TopCoordinates, ZeroTopMotion)
# 	ZeroLegLength = configure.LegLength(ZeroAimTopplate, ServoCoordinates)
# 	ZeroAimTopplate = configure.TopplateMotion(TopCoordinates, ZeroTopMotion)
# 	ZeroAimServoPos = configure.InverseKinematics(ZeroAimTopplate, ServoCoordinates, LINKA, LINKB)
# 	print('ZeroPos', ZeroAimServoPos)

# 	# ZeroTopMotion = [0.1,0.1,0.1,0.0,0.0,0.0] # Angle in radius
# 	# ZeroAimTopplate = configure.TopplateMotion(TopCoordinates, ZeroTopMotion)
# 	# ZeroAimServoPos = configure.InverseKinematics(ZeroAimTopplate, ServoCoordinates, LINKA, LINKB)	

# 	# print(ZeroAimServoPos)
# 	# Monte Carlo
# 	sampleTopplate = configure.MonteCarlo()
# 	for i in range(len(sampleTopplate)):
# 		for j in range(6):
# 			sampleTopplate[i][j] = sampleTopplate[i][j] + ZeroTopMotion[j]
# 	sampleLegLength = [ZeroLegLength]
# 	TopMotionList = [ZeroTopMotion]
# 	AimTopplateList = [ZeroAimTopplate]
# 	AimServoPosList = [ZeroAimServoPos]
# 	for i in range(len(sampleTopplate[0])):
# 		TopMotion = [sampleTopplate[0][i],sampleTopplate[1][i],sampleTopplate[2][i],sampleTopplate[3][i],sampleTopplate[4][i],sampleTopplate[5][i]]
# 		TopMotionList.append(TopMotion)
# 	TopCoordinates = [[70.0000000000026, -24.999999999992898, 208.79999999999063], [69.9999999999976, 25.000000000007095, 208.79999999999563], [-13.349364905396241, 73.12177826490947, 208.80000000000877], [-56.65063509461567, 48.12177826490514, 208.80000000001058], [-56.65063509460605, -48.121778264916266, 208.80000000000098], [-13.349364905381618, -73.12177826491194, 208.79999999999416]]
# 	for i in range(len(TopMotionList)):
# 		TopCoordinates = [[70.0000000000026, -24.999999999992898, 208.79999999999063], [69.9999999999976, 25.000000000007095, 208.79999999999563], [-13.349364905396241, 73.12177826490947, 208.80000000000877], [-56.65063509461567, 48.12177826490514, 208.80000000001058], [-56.65063509460605, -48.121778264916266, 208.80000000000098], [-13.349364905381618, -73.12177826491194, 208.79999999999416]]
# 		AimTopplate = configure.TopplateMotion(TopCoordinates, TopMotionList[i])
# 		InitialCordinates=configure.OriginPosition()
# 		TopCoordinates=InitialCordinates[1]
# 		AimTopplateList.append(AimTopplate)
# 	# Leg Length Analysis 
# 	for i in range(len(AimTopplateList)):
# 		LegLength = configure.LegLength(AimTopplateList[i], ServoCoordinates)
# 		sampleLegLength.append(LegLength)
# 	# Servo Angle Analysis
# 	for i in range(1,len(AimTopplateList)):
# 		AimServoPos = configure.InverseKinematics(AimTopplateList[i], ServoCoordinates, LINKA, LINKB)
# 		TopCoordinates = [[70.0000000000026, -24.999999999992898, 208.79999999999063], [69.9999999999976, 25.000000000007095, 208.79999999999563], [-13.349364905396241, 73.12177826490947, 208.80000000000877], [-56.65063509461567, 48.12177826490514, 208.80000000001058], [-56.65063509460605, -48.121778264916266, 208.80000000000098], [-13.349364905381618, -73.12177826491194, 208.79999999999416]]
# 		InitialCordinates=configure.OriginPosition()
# 		ServoCoordinates=InitialCordinates[2]
# 		AimServoPosList.append(AimServoPos)
# 	# print('Aim Servo Position', AimServoPosList)
# 	sampleServoAngle = [[0],[0],[0],[0],[0],[0]]
# 	for i in range(len(AimServoPosList)):
# 		for j in range(6):
# 			sampleServoAngle[j].append(AimServoPosList[i][j])
# 	# print('Aim Servo Angle Position for each leg', sampleServoAngle)
# 	TempsampleServoAngle = sampleServoAngle
# 	for i in range(6):
# 		sampleServoAngle[i] = sorted(sampleServoAngle[i])
# 	# MC accuracy data analysis
# 	goodCount = [0.0,0.0,0.0,0.0,0.0,0.0]
# 	goodRatio = [0.0,0.0,0.0,0.0,0.0,0.0]
# 	for i in range(6):
# 		for angle in sampleServoAngle[i]:
# 			if angle <= ZeroAimServoPos[i] + 0.5 and angle >= ZeroAimServoPos[i] - 0.5:
# 				goodCount[i] = goodCount[i] + 1.0
# 		goodRatio[i] = goodCount[i] / len(sampleServoAngle[i])
# 	print('Accuracy rate is:' ,goodRatio)
# 	for i in range(6):
# 		sampleServoAngle[i] = sampleServoAngle[i][1:len(sampleServoAngle[i])-1]

# 	# leg 0 handle
# 	minl0 = sampleServoAngle[0][0]
# 	maxl0 = sampleServoAngle[0][len(sampleServoAngle[0])-1]
# 	resolution = (maxl0-minl0) / 1000
# 	leglist = [0]
# 	legcount = [0]
# 	l0 = minl0
# 	i = 0
# 	while l0 < maxl0 and i < len(sampleServoAngle[0])-10:
# 		countl0 = 0
# 		# print(sampleServoAngle[0][i])
# 		while sampleServoAngle[0][i] < (l0 + resolution):
# 			countl0 = countl0+1
# 			i = i + 1
# 		legcount.append(countl0)
# 		leglist.append(l0)
# 		l0 = l0 + resolution
# 	print(len(legcount))
# 	print(len(leglist))
	





# 	# # Normal distribution
# 	# Scount = [0]
# 	# Mlength = np.median(sampleServoAngle[0])
# 	# resolution = 0.01
# 	# limit = 0.6
# 	# Slength = [0]
# 	# print(sampleServoAngle[0][0])


# 	# for i in range(len(sampleServoAngle[0])):
# 	# 	if sampleServoAngle[0][i] <= 

# 	plt.figure(1)               # MC accuracy analysis figure
# 	plt.title('MonteCarlo Accuracy Analysis -- Leg Length Accuracy')
# 	plt.subplot(211)
# 	plt.grid(True) 
# 	plt.ylabel('Topplate Position')
# 	plt.xlabel('Sample Number')
# 	samplePoints = plt.plot(TopMotionList,'.')
# 	plt.setp(samplePoints, color='y')
# 	# plt.axis([170,185,0, len(sampleLegLength)])
# 	plt.subplot(212)
# 	plt.grid(True) 
# 	plt.ylabel('Sample Number')
# 	plt.xlabel('Leg Length/mm')
# 	samplePoints = plt.plot(sampleLegLength,range(len(sampleLegLength)),'.')
# 	plt.setp(samplePoints, color='g')
# 	plt.axis([np.median(sampleLegLength)*0.98,np.median(sampleLegLength)*1.02,0, len(sampleLegLength)])
# 	plt.figure(2)               # MC accuracy analysis figure
# 	plt.title('MonteCarlo Accuracy Analysis -- Servo Angle Accuracy')
# 	for i in range(6):
# 		plt.subplot(611 + i)
# 		plt.grid(True) 
# 		plt.xlabel('Angle-Leg/degree')
# 		samplePoints = plt.plot(sampleServoAngle[i],range(len(sampleServoAngle[i])),'.')
# 		plt.setp(samplePoints, color='r')
# 		plt.axis([sampleServoAngle[i][0], sampleServoAngle[i][len(sampleServoAngle[0])-1], 0, len(sampleServoAngle[i])])
# 	plt.figure(3)
# 	plt.title('Monte-Carlo Accuracy Analysis -- #0 Servo Angle Accuracy')
# 	plt.grid(True) 
# 	plt.ylabel('SampleNumber')
# 	plt.xlabel('Servo Angle')
# 	samplePoints = plt.plot(leglist,legcount,'*')
# 	plt.setp(samplePoints, color='r')
# 	plt.axis([minl0, maxl0, 0, max(legcount)*1.01])

# 	plt.show()

# # 6
# 	# # Forward Kinematics Calculation
# 	# InitialCordinates=configure.OriginPosition()
# 	# BottomCoordinates=InitialCordinates[0]
# 	# TopCoordinates=InitialCordinates[1]
# 	# ServoCoordinates=InitialCordinates[2]
# 	# ZeroTopplate = AimTopplate
# 	# ServoAngle = [25.4388,25.4388,25.4388,25.4388,25.4388,25.4388] # degree
# 	# # ServoAngle = [0.0,0.0,0.0,0.0,0.0,0.0] # degree
# 	# configure.ForwardKinematics(ServoAngle, ServoCoordinates, TopCoordinates, ZeroTopplate, LINKA, LINKB, BOTTOM_ANGLE)

if __name__=='__main__':
    main()