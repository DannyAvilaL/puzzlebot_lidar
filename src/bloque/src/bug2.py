#!/usr/bin/env python

import rospy, tf
import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

scan = None
odom = None
wall_dist = 0.8
w_max = 1
v_max = 1

w = 0
v = 0.1

front_wall = False
right_wall = False

r, l = 0.05, 0.188

goal_x, goal_y = (6.0,-1.0) # El punto debe estar en decimales  (4.0,4.0)
# (-1.0,3.0)
robot_x = 0.0
robot_y = 0.0

def scan_callback(msg):
	global scan
	scan = msg
	scan.ranges = np.array(map(lambda x: x if x != np.inf else scan.range_max, scan.ranges)) 

	#if scan is not None:
		#scan.ranges = np.array(map(lambda x: xif x != np.inf else scan.range_max), scan.ranges) 


def odom_callback(msg):
	global odom, robot_x, robot_y
	if odom is not None:
		robot_x = odom.pose.pose.position.x
		robot_y = odom.pose.pose.position.y
	odom = msg


def turn_to_goal():
	"""
	Function that rotates the robot towards the goal
	"""
	msg = Twist()

	angle_goal = np.arctan2(goal_y - robot_y, goal_x - robot_x)

	orientation_q = odom.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	#falta la transformada en quaterinion.
	angle_error = angle_goal - yaw
	msg.angular.z = angle_error

	print("Turn2goal, error & angulo", angle_error, angle_goal)#, odom.pose.pose.orientation.w)

	vel_pub.publish(msg)
	#00078409461195938501 -> 0.008


	if abs(round(angle_error, 3)) <= 0.1:
		return True
	else:
		return False
	



def calculate_m():
	"""
	Function that calcualtes the slope (m)
	and returns such value
	Argument:
		P: (x, y) tuple
	Returns:
		m: float
	"""
	if goal_x != 0:
		m = (-goal_y)/(-goal_x)
	else: m = 0
	return m

def calculate_line_difference(threshold=0.5):
	"""
	Functions that constantly compares the x position of the robot
	with the x position of the rect line defined at the beginning. 

	If the difference is less than the threshold defined, then it returns True
	otherwise False

	Argument:
		threshold = float
	Returns:
		state: bool
	"""
	m = calculate_m()

	if abs(robot_x*m - robot_y) <= threshold:
		# detiene el right-hand y gira hacia el punto
		return True

	return False


def go_to_wall():
	"""
	Function that makes the robot go forward until
	an obstacle is found.
	"""
	global front_wall
	print('--------------------BUSCANDO MURO ENFRENTE-----------------')
	msg = Twist()
	msg.linear.x = 0.2
	angle_goal = np.arctan2(goal_y - robot_y, goal_x - robot_x)
	orientation_q = odom.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	#falta la transformada en quaterinion.
	angle_error = angle_goal - yaw
	msg.angular.z = angle_error
	vel_pub.publish(msg)

	#front_wall = False
	if scan.ranges[573] < wall_dist:
		#print('muro enfrente')
		
		msg.linear.x = 0
		msg.linear.y = 0
		vel_pub.publish(msg)
		return True
		#msg.angular.z = 0.2
	else:
		
		return False
	

def id_right_wall():
	global right_wall
	right_wall = False
	if scan.ranges[286] < wall_dist + 0.5:
		right_wall = True

	return right_wall
	
def id_left_wall():
	global left_wall
	left_wall = False
	if scan.ranges[859] < wall_dist + 0.5:
		left_wall = True
	
	return left_wall


def range_index(angle):
	min_angle = scan.angle_min
	max_angle = scan.angle_max
	size = len(scan.ranges)
	
	if angle > max_angle:
		return size -1
	elif angle < min_angle:
		return 0
	
	grados = np.linspace(max_angle, min_angle, size)
	index = (np.abs(grados-angle)).argmin()
	return index

def get_distance_in_sector(start_angle, end_angle):

	start_angle = np.radians(start_angle)
	end_angle = np.radians(end_angle)
	
	start_index = range_index(start_angle)
	end_index = range_index(end_angle)
	sector_ranges = scan.ranges[end_index:start_index]
	return np.mean(sector_ranges)

def find_wall_direction(hipotenusa, adyacente):
	"""
	Asume que hay una pared en la derecha
	"""
	hip = get_distance_in_sector(hipotenusa - 1, hipotenusa)
	ady = get_distance_in_sector(adyacente - 1, adyacente)

	alpha = np.arctan2(hip*np.sin(np.deg2rad(hipotenusa))-ady, hip*np.cos(np.deg2rad(hipotenusa))) ########################3

	#print('hip: ' + str(hip) + ' ady: ' + str(ady) + ' alpha: ' + str(alpha))

	return alpha

def position_check():
	
	pass

def follow_right_hand_wall():
	print('--------------------SIGUIENDO MURO-----------------')
	distance_to_right = get_distance_in_sector(88, 91)
	alpha = find_wall_direction(70, 90)
	anguloDerechaDeseado = 0
	distanciaDerechaDeseado = 0.30

	distanciaDerecha = distance_to_right * np.cos(alpha)

	errorAngulo = anguloDerechaDeseado - alpha
	errorDistancia = distanciaDerechaDeseado - distanciaDerecha 

	#print('alpha: ' + str(alpha) + ' distanciaDerecha: ' + str(distanciaDerecha) + ' error angulo: ' + str(errorAngulo) + ' error distancia: ' + str(errorDistancia))
	
	kp_alpha = 0.9
	kp_dist = 1
	if scan.ranges[573] < wall_dist:
		v = 0.1
		w = 0.7
	else:
		v = 0.2
		w = (kp_alpha * errorAngulo) + (kp_dist * errorDistancia)
	
		
	if w > w_max:
		w = w_max
	elif w < -w_max:
		w = -w_max
					
	msg = Twist()
	msg.angular.z = w
	msg.linear.x = v
	vel_pub.publish(msg)

def main():
	global right_wall, front_wall
	msg = Twist()
	# se calcula la pendiente
	m = calculate_m()
	print("Slope: ", m)

	aligned = False
	obstacle_found = False
	follow_wall= False
	iteraciones = 0
	while not rospy.is_shutdown():
		msg = Twist()
		if scan is not None and odom is not None:
			print("Point ({},{})\t Robot ({},{})".format(goal_x, goal_y, robot_x, robot_y))
			# MIENTRAS LA POS DEL ROBOT NO ESTE DEMASIADO CERCA DEL PUNTO P
			if  (round(abs(goal_y - robot_y), 3) >= 0.2 or round(abs(goal_x - robot_x), 3) >= 0.2):
				if obstacle_found and follow_wall:
					print("obstacle_found", obstacle_found, iteraciones)	
					follow_right_hand_wall()
					iteraciones += 1
					if abs(m*robot_x - robot_y) <= 0.5 and iteraciones >= 20000:
						follow_wall = False
					
					
				else:
					obstacle_found = go_to_wall()
					if obstacle_found:
						iteraciones = 0
						follow_wall = True
				
			else:
				print("Arrived to goal")
				msg.angular.z = 0.0
				msg.linear.x = 0.0
				msg.linear.y = 0.0
				
				vel_pub.publish(msg)

	print("Arrived to line")
	msg.angular.z = 0.0
	msg.linear.x = 0.0
	msg.linear.y = 0.0
	
	vel_pub.publish(msg)
if __name__ == '__main__':
	try:
		rospy.init_node('bug2_node')
		scan_listener = rospy.Subscriber('/scan', LaserScan, scan_callback)
		odom_listener = rospy.Subscriber('/odometry',  Odometry, odom_callback)
		vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		rate = rospy.Rate(10)
		main()
	except rospy.ROSInterruptException:
		print("Deteniendo")
		exit()


