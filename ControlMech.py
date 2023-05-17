from asyncio.constants import DEBUG_STACK_DEPTH
import socket
import math
import time
import sys
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
import numpy as np
import threading
import queue
import cv2
import os
import pandas as pd
import numpy as np


center = 0
flag = True
p = 1100
keypoints = 0

def process_frame(frame):
	# It converts the BGR color space of image to HSV color space
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			
	# Threshold of blue in HSV space
	lower_yellow = np.array([0, 140, 140])
	upper_yellow = np.array([40, 255, 255])
			
	# preparing the mask to overlay
	mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
			
	# The black region in the mask has the value of 0,
	# so when multiplied with original image removes all non-blue regions
	result = cv2.bitwise_and(frame, frame, mask = mask)
	# Our operations on the frame come here
			
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	# Display the resulting frame
			
			
	im = result
	gray = cv2.bitwise_not(mask)
	# Set up the detector with default parameters.
	#detector = cv2.SimpleBlobDetector_create()
	params = cv2.SimpleBlobDetector_Params()
	 
	# Change thresholds
	params.minThreshold = 10
	params.maxThreshold = 200
			 
	# Filter by Area.
	params.filterByArea = True
	params.maxArea = 10000000
	params.minArea = 300
	# Filter by Circularity
	params.filterByCircularity = True
	params.minCircularity = 0.35
			 
	# Filter by Convexity
	params.filterByConvexity = False
	params.minConvexity = 0.87
			 
	# Filter by Inertia
	params.filterByInertia = False
	params.minInertiaRatio = 0.01
			 
	# Create a detector with the parameters
	ver = (cv2.__version__).split('.')
	if int(ver[0]) < 3 :
		detector = cv2.SimpleBlobDetector(params)
	else : 
		detector = cv2.SimpleBlobDetector_create(params)
			
	# Detect blobs.
	global keypoints 
	keypoints = detector.detect(gray)
			
	im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
	#im_with_keypoints = cv2.drawKeypoints(mask, merged_keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
	return im_with_keypoints
	# Show keypoints

def display_frames(queue):
    while True:
        frame = queue.get()
        if frame is None:
            break
        cv2.imshow("Keypoints", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
	
def compute_object_position(x, y, alpha, D, theta):
	#Global coords of robot considering angle
	alpha = math.radians(alpha)

	R = np.array([[math.cos(theta), -math.sin(theta), x],
              [math.sin(theta), math.cos(theta), y],
              [0, 0, 1]])

	target_local = np.array([[D * math.cos(alpha)], [D * math.sin(alpha)], [1]])

	target_global = np.matmul(R, target_local)

	#Duck coords actual
	Duck_x_global = target_global[0][0]
	Duck_y_global = target_global[1][0]
	return Duck_x_global, Duck_y_global
	

class PID:
    def __init__(self, Kp, Ki, Kd, windup_guard=20):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.windup_guard = windup_guard
        self.last_error = 0
        self.integral = 0

    def update(self, error, delta_time):
        # Calculate the derivative
        if delta_time > 0:
            derivative = (error - self.last_error) / delta_time
        else:
            derivative = 0

        # Update the integral with saturation (anti-windup)
        self.integral += error * delta_time
        self.integral = np.clip(self.integral, -self.windup_guard, self.windup_guard)

        # Calculate the control output
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Update the last error
        self.last_error = error

        return output

class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.prev_value = None

    def update(self, value):
        if self.prev_value is None:
            self.prev_value = value
        else:
            self.prev_value = self.alpha * value + (1 - self.alpha) * self.prev_value
        return self.prev_value

def wrap_angle_rad(angle_rad):
    return (angle_rad + math.pi) % (2 * math.pi) - math.pi


def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    positions[robot_id] = position
    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)
    rotations[robot_id] = rotz



# Create low-pass filters for position measurements
lpf_x = LowPassFilter(alpha=0.8)
lpf_y = LowPassFilter(alpha=0.8)


def detection():
    N = 2
    i = 0
    print('detection')
    while True:
        try:
            ret, frame = cap.read()
            print('frame')
            # Create a thread pool
            processed_frame = process_frame(frame)
            queue.put(processed_frame)
            
            for keypoint in keypoints:
                x = keypoint.pt[0]
                y = keypoint.pt[1]
                z = keypoint.size
                
                D = (757 * (1 / z) + 1.57) * 0.0254  # meters conversion
                Theta = -((1.49 * 10 ** -3) * x - 0.498)
                ox, oy = compute_object_position(positions[robot_id][0], positions[robot_id][1], rotations[robot_id], D, Theta)
                duck_waypoint = (ox, oy)
                print(f"{positions[robot_id][0]:.2f}, {positions[robot_id][1]:.2f}, {rotations[robot_id]:.2f}, {ox:.2f}, {oy:.2f}",end='\r')
                i+=1
            display_frames(q)
        except BaseException as e:
            break


def mapping(target_x, target_y, clientSocket):
    print('mapping')
    tolerance = 0.1
    error_position = 1
    pid_translation = PID(Kp=100, Ki=20, Kd=1)
    pid_rotation = PID(Kp=100, Ki=10, Kd=0.5)

    print(target_x, target_y)
    while error_position > tolerance:
        #pos_x_filtered = lpf_x.update(positions[robot_id][0])
        #pos_y_filtered = lpf_y.update(positions[robot_id][1])
        print('pos')
        #errx = target_x - pos_x_filtered
        #erry = target_y - pos_y_filtered
        print(target_x)
        print(positions[robot_id][0])
        errx = target_x - positions[robot_id][0]
        erry = target_y - positions[robot_id][1]
        
        # Calculate control input for orientation and translation
        delta_time = 0.05
        error_position = math.sqrt(errx ** 2 + erry ** 2)
        control_signal_translation = pid_translation.update(error_position, delta_time)
        print('error calc')
        # Calculate the angle between the desired direction and the current orientation
        alpha_p = wrap_angle_rad(math.atan2(erry, errx))
        beta = wrap_angle_rad(alpha_p - rotations[robot_id] * math.pi / 180)

        # Update the rotation PID controller
        control_signal_rotation = pid_rotation.update(beta, delta_time)
        print('control')
        # Calculate translational control input components
        v_x = control_signal_translation * math.cos(beta)
        v_y = control_signal_translation * math.sin(beta)

        fl = v_x - v_y - control_signal_rotation
        fr = v_x + v_y + control_signal_rotation
        bl = v_x - v_y + control_signal_rotation
        br = v_x + v_y - control_signal_rotation

        u = np.array([fl, bl, fr, br])
        print('u')
        # Find the minimum motor speed among the absolute values of motor speeds
        min_motor_speed = np.min(np.abs(u))

        # If the minimum motor speed is less than the scaling factor m, scale the motor speeds accordingly
        m = 600
        if min_motor_speed < m:
            scaling_factor = m / min_motor_speed
            u = u * scaling_factor + np.sign(u) * m

        # Ensure that the motor speeds are still within the desired range
        u[u > 1500] = 1500
        u[u < -1500] = -1500

        #print("U after adjustment: ", u)
        print('move')
        command = 'CMD_MOTOR#%d#%d#%d#%d\n' % (int(u[0]), int(u[1]), int(u[2]), int(u[3]))
        clientSocket.send(command.encode('utf-8'))
        print("Position: ", positions[robot_id][0], positions[robot_id][1])
        time.sleep(0.5)
    
    print('arrived')
    
    


def spinning(clientSocket):
    s = clientSocket
    i = 0
    duck_detected = False
    while i < 360 and not duck_detected:
        print('spinning')
        command = 'CMD_MOTOR#%d#%d#%d#%d\n' % (int(800), int(800), int(-800), int(-800))
        s.send(command.encode('utf-8'))
        print(i)
        detection()
        global ox, oy
        ox = -30
        oy = -30
        if len(keypoints) > 0:
            # If a duck is detected, get the duck's location
            #for keypoint in keypoints:
            x = keypoint.pt[0]
            y = keypoint.pt[1]
            z = keypoint.size
                    
            D = (757 * (1 / z) + 1.57) * 0.0254  # meters conversion
            Theta = -((1.49 * 10 ** -3) * x - 0.498)
            ox, oy = compute_object_position(pos_x_filtered, pos_y_filtered, rotations[robot_id], D, Theta)
            duck_detected = True
            break
        i += 1
        
def upDuck(target_x, target_y, clientSocket):
    print('getting duck')
    tolerance = 0.1
    error_position = 1
    pid_translation = PID(Kp=50, Ki=20, Kd=1)
    pid_rotation = PID(Kp=100, Ki=10, Kd=0.5)

    print(target_x, target_y)
    while error_position > tolerance:
        #pos_x_filtered = lpf_x.update(positions[robot_id][0])
        #pos_y_filtered = lpf_y.update(positions[robot_id][1])
        print('pos')
        #errx = target_x - pos_x_filtered
        #erry = target_y - pos_y_filtered
        errx = target_x - positions[robot_id][0]
        erry = target_y - positions[robot_id][1]
        print('error')
        # Calculate control input for orientation and translation
        delta_time = 0.05
        error_position = math.sqrt(errx ** 2 + erry ** 2)
        control_signal_translation = pid_translation.update(error_position, delta_time)
        print('error calc')
        # Calculate the angle between the desired direction and the current orientation
        alpha_p = wrap_angle_rad(math.atan2(erry, errx))
        beta = wrap_angle_rad(alpha_p - rotations[robot_id] * math.pi / 180)

        # Update the rotation PID controller
        control_signal_rotation = pid_rotation.update(beta, delta_time)
        print('control')
        # Calculate translational control input components
        v_x = control_signal_translation * math.cos(beta)
        v_y = control_signal_translation * math.sin(beta)

        fl = v_x - v_y - control_signal_rotation
        fr = v_x + v_y + control_signal_rotation
        bl = v_x - v_y + control_signal_rotation
        br = v_x + v_y - control_signal_rotation

        u = np.array([fl, bl, fr, br])
        print('u')
        # Find the minimum motor speed among the absolute values of motor speeds
        min_motor_speed = np.min(np.abs(u))

        # If the minimum motor speed is less than the scaling factor m, scale the motor speeds accordingly
        m = 600
        if min_motor_speed < m:
            scaling_factor = m / min_motor_speed
            u = u * scaling_factor + np.sign(u) * m

        # Ensure that the motor speeds are still within the desired range
        u[u > 1500] = 1500
        u[u < -1500] = -1500

        #print("U after adjustment: ", u)
        print('move')
        command = 'CMD_MOTOR#%d#%d#%d#%d\n' % (int(u[0]), int(u[1]), int(u[2]), int(u[3]))
        clientSocket.send(command.encode('utf-8'))
        print("Position: ", positions[robot_id][0], positions[robot_id][1])
        time.sleep(0.5)
    
    print('arrived')
        

        



def lift(client_socket):
    
    rotate = 'CMD_SERVO#%d#%d\n' %(int(1), int(50))
    client_socket.send(rotate.encode('utf-8'))
    time.sleep(1)
    
    rotate = 'CMD_SERVO#%d#%d\n' %(int(0), int(0))
    client_socket.send(rotate.encode('utf-8'))
    time.sleep(1)
    


def drop(client_socket):

    rotate = 'CMD_SERVO#%d#%d\n' %(int(1), int(120))
    client_socket.send(rotate.encode('utf-8'))
    time.sleep(1)
    rotate = 'CMD_SERVO#%d#%d\n' % (int(0), int(80))
    client_socket.send(rotate.encode('utf-8'))
    time.sleep(1)



# Set the initial motor speeds to zero
speed_x = 0
speed_y = 0
speed_rot = 0

# Initialize position and rotation data
positions = {}
rotations = {}


if __name__ == "__main__":
    try:
        clientAddress = "192.168.0.29"
        optitrackServerAddress = "192.168.0.4"
        #global robot_id
        robot_id = 302
        print('ip')
        # Connect to the robot
        IP_ADDRESS = '192.168.0.202'
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((IP_ADDRESS, 5000))
        print('Connected')

        # This will create a new NatNet client
        streaming_client = NatNetClient()
        print('1')
        streaming_client.set_client_address(clientAddress)
        print('2')
        streaming_client.set_server_address(optitrackServerAddress)
        print('3')
        streaming_client.set_use_multicast(True)
        print('4')
        # Configure the streaming client to call our rigid body handler on the emulator to send data out.
        streaming_client.rigid_body_listener = receive_rigid_body_frame
        print("optitrack Online")
        # Start up the streaming client now that the callbacks are set up.
        # This will run perpetually, and operate on a separate thread.
        '''
        global cap
        global q
        cap = cv2.VideoCapture('http://192.168.0.202:3141/stream.mjpg')
        if not cap.isOpened():
            print("Error opening video stream or file")
            exit()
        print('import cap')
        '''
        is_running = streaming_client.run()
        '''
        q = queue.Queue()
        detection_thread = threading.Thread(target=detection, args=(q,cap))
        detection_thread.start()
        print('queue online')
        '''
        time.sleep(0.5)
    except BaseException as e: 
        print('failed to connect')
        command = 'CMD_MOTOR#%d#%d#%d#%d\n' % (0,0,0,0)
        s.send(command.encode('utf-8'))
        time.sleep(0.5)
        streaming_client.shutdown()
        s.shutdown(2)
        s.close()
       



    
    command = 'CMD_MOTOR#%d#%d#%d#%d\n' % (0,0,0,0)
    s.send(command.encode('utf-8'))
    print('socket test')

    try:
        print('going')
        #global ox, oy
        ox = -30
        oy = -30
        drop(s)
        #mapping(float(-3.01), float(0.15), s)
        time.sleep(3)
        lift(s)
        #mapping(float
        #(5.07), float(0.16), s)
        time.sleep(3)
        drop(s)
        '''
        waypoints = [(-3.01, 0.15), (5.07, 0.16), (-3.39, 1.72), (-1.5, 2.5), (1.29, 0.63), (5.5, 0.05)]
        for waypoint in waypoints:
            mapping(*waypoint, s)
        '''
        ''' 
        if ox != -30 and oy != -30:
        upDuck(ox, oy, s, cap, q)
        '''

                

        
        '''
        #demonstration claw
        lift(s)
        time.sleep(3)
        #mapping(0,0)
        drop(s)
        '''
        
    except BaseException as e:
        # STOP
        print('overall')
        command = 'CMD_MOTOR#%d#%d#%d#%d\n' % (0, 0, 0, 0)
        s.send(command.encode('utf-8'))
        time.sleep(0.5)
        streaming_client.shutdown()
        s.shutdown(2)

    finally:
        # Clean up the resources and send the motor stop command
        command = 'CMD_MOTOR#%d#%d#%d#%d\n' % (0, 0, 0, 0)
        s.send(command.encode('utf-8'))
        time.sleep(0.5)
        streaming_client.shutdown()
        s.shutdown(2)
        s.close()
