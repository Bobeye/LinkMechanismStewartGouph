# Wind Tunnel Control
import math
import time
import serial
import serial.tools.list_ports

def WIN_CONNECT_WINDTUNNEL():
	PortsList = list(serial.tools.list_ports.comports())
	PortsNum = len(PortsList)
	for i in range(PortsNum):
		PortsList[i] = str(PortsList[i])
		if "Arduino" in PortsList[i]:
			print ("COM Sequence:",i,"Device Info:",PortsList[i])
		else:
			print ("No Arduino Detected")
	COMtoConnect = input("Choose the right COM sequence for wind tunnrl control...")
	COMtoConnect = int(COMtoConnect)
	print("Trying to connect to", PortsList[COMtoConnect][:4])
	WTport = serial.Serial(PortsList[COMtoConnect][:4], 9600)

def LINUX_CONNECT_WINDTUNNEL():
	PortsList = list(serial.tools.list_ports.comports())
	PortsNum = len(PortsList)
	for i in range(PortsNum):
		print("COM Sequence:",i,"Device Info:",PortsList[i])
	COMtoConnect = input("Choose the right COM sequence for wind tunnrl control...")
	COMtoConnect = int(COMtoConnect)
	print("Trying to connect to", PortsList[COMtoConnect])


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
	WIN_CONNECT_WINDTUNNEL()

