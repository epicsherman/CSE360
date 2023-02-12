from Motor import *
from Ultrasonic import *
import math
import time

PWM = Motor()
US = Ultrasonic()

TARGET = 50

# k-values we tried: 40, 200, 1000

def test():
	try:
		speed = 0
		k = 200
		#PWM.setMotorModel(speed,1000,1000,1000) # forward
		print("Starting")
		#for i in range(100):
		#	US.get_distance()
		time.sleep(3)
		while True:
			print("enter iter")
			PWM.setMotorModel(speed, speed, speed, speed)
			dist = US.get_distance()
			dp = dist - TARGET
			print(dist, dp)
			speed = int(k * dp)
			#if abs(dp) < 2:
			#	break
		PWM.setMotorModel(0,0,0,0) # stop
		print("End, normal")
	except:
		PWM.setMotorModel(0,0,0,0) # stop
		print("End, abrupt")

if __name__ == '__main__':
    test()

        
