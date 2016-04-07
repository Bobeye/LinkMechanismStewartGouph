# Wind Tunnel Control
import math
import time
import serial
import serial.tools.list_ports

def CONNECT_WINDTUNNEL():
	PortsList = list(serial.tools.list_ports.comports())
	print(PortsList)
	if "Arduino" in PortsList
# for p in PortsList:

# if "Arduino" in p[1]:
#         print "This is an Arduino!"



# WTport = serial.Serial('COM9', 9600)
# WTport.write('(',890,')')

# print("Wind Tunnel Connected")


# Wind Speed Lookup Table
# 890	8
# 1020	6.85
# 1175	6
# 1350	4.78
def WIND(windspeed):
	print("Current Wind Speed: ", windspeed)



if __name__=='__main__':
	CONNECT_WINDTUNNEL()

