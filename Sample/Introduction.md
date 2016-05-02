Introduction of the Stewart Platform Operation with Python

_____________________________________________________________________________
--configure.py is the structure and kinematics setup for STP
--inversekinematics.py transfers the given motion of the top plate into the requied servo angle
--movestewart.py controls the motion of the STP

For controlling dynamixel ax-12a servo with python, please install pypot first:
```
  pip install pypot
```
___________________________________________________________________________________
1. Confifure the STP
```python
  import configure as CO
  configure = CO.CONFIGURE()
  InitialCordinates = configure.OriginPosition()
  CO_BottomC=InitialCordinates[0] # Initial bottom plate coordinates
	CO_TopC=InitialCordinates[1]    # Initial top plate coordinates
	CO_ServoC=InitialCordinates[2]  # Initial servo coordinates
```

________________________________________________________________________________________
2. Inverse Kinematics
```python
  import configure as CO
  AimTopplate = configure.TopplateMotion(CO_TopC, TopMotion) # Topplate rotation & translation
	AimServoPos = configure.InverseKinematics(AimTopplate, CO_ServoC, CO.LINKA, CO.LINKB) # inverse kinematics
```
