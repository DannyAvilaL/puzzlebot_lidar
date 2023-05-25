#!/usr/bin/env python

import rospy, tf
import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

scan = None
odom = None

WALL_DIST = 0.5
W_MAX = 0.4
V_MAX = 0.3
R, L = 0.05, 0.188

w = 0
v = 0.1

front_wall = False
right_wall = False

# Goal must be in decimals
goal_list = [(-2.0, 3.0), (-1.0, 8.0), (1.0, 3.0), (0.0, 0.0)]

goal_x, goal_y = (-2.0, 4.0) 
robot_x = 0.0
robot_y = 0.0

def get_new_goal():
	for i in goal_list:
		yield i

def scan_callback(msg):
	global scan
	scan = msg
	if scan is not None:
		scan.ranges = np.array(map(lambda x: x if x != np.inf else scan.range_max, scan.ranges)) 


def odom_callback(msg):
	global odom, robot_x, robot_y
	if odom is not None:
		robot_x = odom.pose.pose.position.x
		robot_y = odom.pose.pose.position.y
	odom = msg


def turn_to_goal():
	"""
	Function that rotates the robot towards the goal
	Returns True unless is heading towards goal
	"""
	msg = Twist()

	angle_goal = np.arctan2(goal_y - robot_y, goal_x - robot_x)

	orientation_q = odom.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	#falta la transformada en quaterinion.
	angle_error = angle_goal - yaw
	if abs(round(angle_error, 3)) <= 0.1:
		return angle_error, True
	else:
		return angle_error, False


	

def near_obstacle(dist=WALL_DIST):
	"""
	Return True if an obstacle is in front of the object
	Args:
		dist: certain distance defined
	Returns:
		boolean
	"""
	if get_distance_in_sector(-10, 0) <= dist or get_distance_in_sector(0, 10) <= dist:
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
	else: 
		m = 0

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
	msg.linear.x = 0.1
	angle_goal = np.arctan2(goal_y - robot_y, goal_x - robot_x)
	orientation_q = odom.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	#falta la transformada en quaterinion.
	angle_error = angle_goal - yaw

	if abs(angle_error) >= 35:
		msg.linear.x = 0

	if angle_error >= W_MAX:
		angle_error = W_MAX
	elif angle_error <= -W_MAX:
		angle_error = -W_MAX

	msg.angular.z = angle_error
	vel_pub.publish(msg)

	if get_distance_in_sector(-15, 15) < WALL_DIST:
		#print('muro enfrente')
		msg.linear.x = 0
		msg.linear.y = 0
		msg.angular.z = 0.2
		vel_pub.publish(msg)
		return True
	else:		
		return False


def range_index(angle):
	"""
	Function that returns the index of the angle
	closest to the angle given.
	Args: 
		angle: angle in float
	Returns:
	- index: int
	"""
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
	"""
	Function that returns a mean of ranges between
	a certain range in the LiDER values.
	Argument:
	- start_angle: float
	- end_angle: float
	Returns:
	- mean: float
	"""
	start_angle = np.radians(start_angle)
	end_angle = np.radians(end_angle)
	start_index = range_index(start_angle)
	end_index = range_index(end_angle)
	sector_ranges = scan.ranges[end_index:start_index]
	return np.mean(sector_ranges)


def find_wall_direction(hip, lat, dir):
	"""
	Asume que hay una pared en la derecha
	"""
	hip = get_distance_in_sector(hip - 2, hip)
	ady = get_distance_in_sector(lat - 2, lat)
	if dir == "left":
		alpha = np.arctan2(hip*np.cos(np.deg2rad(abs(hip)))-ady,
		    		   hip*np.sin(np.deg2rad(abs(lat)))) 
	elif dir == "right":
		alpha = np.arctan2(hip*np.sin(np.deg2rad(abs(hip)))-ady,
		    		   	hip*np.cos(np.deg2rad(abs(hip)))) 
	return alpha


def side_check():
	"""
	Decides the angle of the obstacle in the front
	so decides wether do right follower or left follower
	"""

	left_side = get_distance_in_sector(-80, -10)
	right_side = get_distance_in_sector(10, 80)

	if right_side >= left_side:
		return "left"
	else:
		return "right"


def follow_right_hand_wall():
	#print('FOLLOWING FROM RIGHT')
	kp_alpha = 2
	kp_dist = 1
	distance_to_right = get_distance_in_sector(88, 91)
	alpha = find_wall_direction(70, 90, "right")
	anguloDerechaDeseado = 0
	distanciaDerechaDeseado = 0.1

	distanciaDerecha = distance_to_right * np.cos(alpha)

	errorAngulo = anguloDerechaDeseado - alpha
	errorDistancia = distanciaDerechaDeseado - distanciaDerecha 
	#print("ERROR DISTANCIA", errorDistancia)
	if get_distance_in_sector(-5, 5) < WALL_DIST:
		print("WALL AT THE FRONT")
		v = 0
		w = 0.5
	else:
		v = 0.05
		w = (kp_alpha * errorAngulo) + (kp_dist * errorDistancia)
	
	if w >= W_MAX:
		w = W_MAX
	elif w <= -W_MAX:
		w = -W_MAX
					
	msg = Twist()
	msg.angular.z = w
	msg.linear.x = v
	vel_pub.publish(msg)


def follow_left_hand():
	kp_alpha = 3
	kp_dist = 2

	anguloDerechaDeseado = 0
	distanciaDerechaDeseado = -0.1

	alpha = find_wall_direction(-45, -90, "left")

	distance_to_right = get_distance_in_sector(-90, -86)
	#print("DISTANCE", distance_to_right, np.cos(alpha))
	distanciaDerecha = distance_to_right * np.cos(alpha)
	errorAngulo = anguloDerechaDeseado + alpha
	errorDistancia = distanciaDerechaDeseado + distanciaDerecha
	#print("ERROR DISTANCIA", errorDistancia)

	if get_distance_in_sector(-5, 5) < WALL_DIST:
		print("WALL AT THE FRONT")
		v = 0
		w = -0.5
	else:
		v = 0.05
		w = (kp_alpha * errorAngulo) + (kp_dist * errorDistancia)
		#print(kp_alpha, errorAngulo, kp_dist, errorDistancia)
		#print("====")

	#saturacion
	if w >= W_MAX:
		w = W_MAX
	elif w <= -W_MAX:
		w = -W_MAX
	
	#rospy.loginfo("{} {} {}".format(errorAngulo, errorDistancia, w))
					
	msg = Twist()
	msg.angular.z = w
	msg.linear.x = v
	#print(msg)
	vel_pub.publish(msg)


def main():
	global right_wall, front_wall, goal_y, goal_x
	msg = Twist()
	# se calcula la pendiente
	m = calculate_m()
	print("Slope: ", m)
	lado_seleccionado = False
	obstacle_found = False
	towards_goal = False
	follow_wall= False
	iteraciones = 0
	puntos = get_new_goal()
	try:
		goal_x, goal_y = next(puntos)
		while not rospy.is_shutdown():
			msg = Twist()
			if scan is not None and odom is not None:
				print("Point ({},{})\t Robot ({},{})".format(goal_x, goal_y, robot_x, robot_y))
				# MIENTRAS LA POS DEL ROBOT NO ESTE DEMASIADO CERCA DEL PUNTO P
				robot_angle = turn_to_goal()
				if (round(abs(goal_y - robot_y), 3) >= 0.1 or round(abs(goal_x - robot_x), 3) >= 0.1):
					if obstacle_found and follow_wall:
						#print("obstacle_found", obstacle_found, "angulo", robot_angle)	
						if not lado_seleccionado:
							print("SELECCIONADO LADO")
							lado_seleccionado = side_check()
							t0 = rospy.Time.now().to_sec()	
							follow_wall = True
						else:
							if lado_seleccionado == "right" and follow_wall:
								#print("RIGHT HAND", robot_angle[0])
								#follow_left_hand()
								follow_right_hand_wall()
								angle_error, _ = robot_angle
							elif lado_seleccionado == "left" and follow_wall:
								#print("LEFT HAND", robot_angle[0])
								#follow_right_hand_wall()
								follow_left_hand()
								
								angle_error, _ = robot_angle

							#print("ERROR DE ANGULO", angle_error)
							tiempo = rospy.Time.now().to_sec() - t0

							# si esta apuntando al mismo lado del goal
							print(abs(angle_error) <= 0.2, tiempo > 3)
							if abs(angle_error) <= 0.2 and tiempo > 3: 
								print("VIENDO A GOAL, SALIENDO DE FOLLOW")
								if (lado_seleccionado == "right" and get_distance_in_sector(-30, 0) >= 1.5) or \
									(lado_seleccionado == "left" and get_distance_in_sector(0, 30) >= 1.5):
									follow_wall = False

					else:
						obstacle_found = go_to_wall()

						if obstacle_found or near_obstacle():
							iteraciones = 0
							follow_wall = True
						#		pass
					
				else:
					print("Arrived to goal")
					#msg.angular.z = 0.1
					#msg.linear.x = 0.0
					#msg.linear.y = 0.0
					
					#vel_pub.publish(msg)
					print("Getting new goal")
					goal_x, goal_y =  next(puntos)

			#rate.sleep()
	except StopIteration:
		print("Completed map points")
		msg.angular.z = 0.0
		msg.linear.x = 0.0
		msg.linear.y = 0.0
		vel_pub.publish(msg)
		print("Restarting points")
		puntos = get_new_goal()


if __name__ == '__main__':
	try:
		rospy.init_node('bug2_node')
		scan_listener = rospy.Subscriber('/scan', LaserScan, scan_callback)
		odom_listener = rospy.Subscriber('/odom',  Odometry, odom_callback)
		vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		rate = rospy.Rate(3)
		main()
	except rospy.ROSInterruptException:
		print("Deteniendo")
		exit()
