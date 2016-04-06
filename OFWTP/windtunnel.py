# Wind Tunnel Control
import math
import time
import serial

WTport = serial.Serial('COM9', 9600)
WTport.write('(',890,')')


# Wind Speed Lookup Table
# 890	8
# 1020	6.85
# 1175	6
# 1350	4.78
def WIND(windspeed):
	print("Current Wind Speed: ", windspeed)
