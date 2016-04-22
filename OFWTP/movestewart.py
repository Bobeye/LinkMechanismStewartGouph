import dynamixel

# Motor configuration
ports = dynamixel.get_available_ports()
if not ports:
	raise IOError('no port found!')
print('ports found', ports)
print('connecting on the first available port:', ports[0])
dxl_io = dynamixel.DxlIO(ports[0])
Motors = dxl_io.scan(range(10))
print('Motors ID', Motors)
##########Speed Set#####################
# for i in Motors:
# 	dxl_io.set_moving_speed({i: 1000})
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



main()