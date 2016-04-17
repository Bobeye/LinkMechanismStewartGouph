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
	return PortsList[COMtoConnect][:4]
	

def LINUX_CONNECT_WINDTUNNEL():
	PortsList = list(serial.tools.list_ports.comports())
	PortsNum = len(PortsList)
	for i in range(PortsNum):
		print("COM Sequence:",i,"Device Info:",PortsList[i])
	COMtoConnect = input("Choose the right COM sequence for wind tunnrl control...")
	COMtoConnect = int(COMtoConnect)
	print("Trying to connect to", PortsList[COMtoConnect][0])


# Wind Speed Lookup Table
# 890	8
# 1020	6.85
# 1175	6
# 1350	4.78
# WTport = serial.Serial(PortsList[COMtoConnect][:4], 9600)

def WIND(windspeed):

	print("Current Wind Speed: ", windspeed)



if __name__=='__main__':
	WTport = WIN_CONNECT_WINDTUNNEL()
	ControlWTport = serial.Serial(WTport, 9600)
	print("Wind Turbine Connected")
	for i in range(2):
		a = "(2000)"
		ControlWTport.write(a)
		# ControlWTport.write('(')
		# ControlWTport.write('890')
		# ControlWTport.write(')')	
		time.sleep(1)
		print("giving wind speed command")


