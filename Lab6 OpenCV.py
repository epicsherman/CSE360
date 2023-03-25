### this code works only on the raspberry
import numpy as np
import cv2
from Motor import *
import math
PWM = Motor()


from picamera2 import Picamera2 
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()
center = 0
flag = True
p = 1100
while True:
	try:
		if flag == False:
			break
		PWM.setMotorModel(-p,-p,p,p)
		frame = picam2.capture_array()
		# It converts the BGR color space of image to HSV color space
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		  
		# Threshold of blue in HSV space
		lower_blue = np.array([90, 130, 130])
		upper_blue = np.array([120, 255, 255])

		# preparing the mask to overlay
		mask = cv2.inRange(hsv, lower_blue, upper_blue)
		  
		# The black region in the mask has the value of 0,
		# so when multiplied with original image removes all non-blue regions
		result = cv2.bitwise_and(frame, frame, mask = mask)
		# Our operations on the frame come here
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		# Display the resulting frame
		#cv2.imshow('frame',gray)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
		
		
		src = result
		gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
		gray = cv2.medianBlur(gray, 5)
		rows = gray.shape[0]
		circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows/8, param1=100, param2=30, minRadius=1, maxRadius=1000)
		if circles is not None:
			circles = np.uint16(np.around(circles))
			for i in circles[0, :]:
				center = (i[0], i[1])
				# circle center
				cv2.circle(src, center, 1, (0, 100, 100), 3)
				# circle outline
				radius = i[2]
				cv2.circle(src, center, radius, (0, 0, 255), 3)
				#print(src.shape)
				print(center)
				#center is 320
				
				#PID
				theta = center[0] - 320
				print(theta)
				Kt = 2
				v =  0 #Kv * error_position
				omega = -Kt *theta
				u = np.array([v - omega, v + omega])
				u[u > 1500] = 1500
				u[u < -1500] = -1500
				PWM.setMotorModel(int(u[0]),int(u[0]),int(u[1]),int(u[1]))
				time.sleep(0.1)
				
				if (abs(theta) < 6):
					PWM.setMotorModel(0,0,0,0)
					flag = false
					break
		if flag == False:
			break
		cv2.imshow('frame', frame)
		#cv2.imshow('mask', mask)
		#cv2.imshow('result', result)
		cv2.imshow("detected circles", src)
	except :
		PWM.setMotorModel(0,0,0,0)
		break
# When everything done, release the capture
cv2.destroyAllWindows()

