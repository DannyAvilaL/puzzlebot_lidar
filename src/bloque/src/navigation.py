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
WALL_FRONT_DIST = 1
W_MAX = 1
V_MAX = 0.1

w = 0
v = 0.1

front_wall = False
right_wall = False
t0 , t = 0,0

# Goal must be in decimals
goal_list = [(2.0, 3.0), (5.0, -3.0), (9.0, 1.0), (3.0, 3.0), (0.0, 0.0)]

goal_x, goal_y = (-2.0, 4.0) 
robot_x = 0.0
robot_y = 0.0
robot_q = 0.0

states  = ["turn_to_goal", "go_to_goal", "select_side", "obstacle_right", "obstacle_left", "get_goal", "stop"]

def get_new_goal():
	for i in goal_list:
		yield i

def scan_callback(msg):
	global scan
	scan = msg
	if scan is not None:
		scan.ranges = np.array(map(lambda x: x if x != np.inf else scan.range_max, scan.ranges)) 


def odom_callback(msg):
	global odom, robot_x, robot_y, robot_q
	odom = msg
	if odom is not None:
		robot_x = odom.pose.pose.position.x
		robot_y = odom.pose.pose.position.y
		q = odom.pose.pose.orientation
		(roll, pitch, theta) = euler_from_quaternion([q.x, q.y, q.z, q.w])
		robot_q = theta

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
	#dist = np.sqrt((grados-angle)**2)
	#index = np.argmin(dist)
	#index = np.argmin()
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
	#print(start_index, end_index)
	return np.mean(sector_ranges)


def find_wall_directionunused(hip, lat, dir):
	"""
	Asume que hay una pared en la derecha
	"""
	hip = get_distance_in_sector(hip - 1, hip)
	ady = get_distance_in_sector(lat - 1, lat)
	if dir == "left":
		alpha = np.arctan2(hip*np.cos(np.deg2rad(abs(hip)))-ady,
		    		   hip*np.sin(np.deg2rad(abs(lat)))) 
	elif dir == "right":
		alpha = np.arctan2(hip*np.sin(np.deg2rad(abs(hip)))-ady,
		    		   	hip*np.cos(np.deg2rad(abs(hip)))) 
	return alpha


def find_wall_direction_der(hipotenusa, adyacente):
	"""
	Asume que hay una pared en la derecha
	"""
	hip = get_distance_in_sector(hipotenusa - 1, hipotenusa)
	ady = get_distance_in_sector(adyacente - 1, adyacente)

	alpha = np.arctan2(hip*np.sin(np.deg2rad(hipotenusa))-ady, 
		    hip*np.cos(np.deg2rad(hipotenusa))) ########################3

	#print('hip: ' + str(hip) + ' ady: ' + str(ady) + ' alpha: ' + str(alpha))

	return alpha


def find_wall_direction_ziq():
    """Assuming wall is on the right, finds the direction of the wall w.r.t
    the robot frame (x ahead, y to the left). The direction is returned
    as an angle, which is 0 if the wall is parallel to the heading of the
    robot and negative if the robot is heading away from the wall.

    Tests
    -----
    >>> scan = generate_test_scan(straight_wall=True)
    >>> wall_dir = find_wall_direction(scan)
    >>> np.abs(wall_dir) < 1e-6
    True
    """

    #hip = get_distance_in_sector(scan, np.radians(45-1), np.radians(45))
    #ady = get_distance_in_sector(scan, np.radians(90-1), np.radians(90))

    hip = get_distance_in_sector(-46, -44)
    ady = get_distance_in_sector(-90, -86)


    alpha = np.arctan2(hip * np.cos(np.deg2rad(45))-ady, 
                       hip*np.sin(np.deg2rad(90)))
    #print(hip, ady, alpha, "FIND WALL")
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
	

### FUNCIONES DE ESTADOS
def turn_to_goal():
	"""
	Function that rotates the robot towards the goal
	within the same (x,y) position
	"""
	msg = Twist()
	angle_goal = np.arctan2(goal_y - robot_y, goal_x - robot_x)
	angle_error = angle_goal - robot_q

	w = angle_error * 0.02
	if angle_error >= 0.03:
		w = W_MAX
	elif angle_error <= -0.03:
		w = -W_MAX

	msg.angular.z = w
	vel_pub.publish(msg)
	# deleting the message
	del msg
	# check if the robot is heading towards goal
	if abs(round(angle_error, 3)) <= 0.1:
		return True
	else:
		return False


def get_robot_angle():
	angle_goal = np.arctan2(goal_y - robot_y, goal_x - robot_x)

	orientation_q = odom.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	#falta la transformada en quaterinion.
	angle_error = angle_goal - yaw
	return angle_error

def go_to_goal():
	"""
	Makes the robot go forward until an obstacle is found.
	"""
	global front_wall
	kp = 0.6
	kv = 0.1
	msg = Twist()

	dist = 0.05

	angle_goal = np.arctan2(goal_y - robot_y, goal_x - robot_x)
	d = np.sqrt((goal_y-robot_y)**2 + (goal_x - robot_x)**2)
	print("DISTANCIA", d)

	angle_error = (angle_goal - robot_q) * kp
	dist_error = (d - dist) * kv
	
	w = angle_error
	v = dist_error
	if angle_error >= 0.08:
		w = W_MAX
	elif angle_error <= -0.08:
		w = -W_MAX

	if dist_error >= V_MAX:
		v = V_MAX
	elif dist_error <= -V_MAX:
		v = -V_MAX

	distancia_abanico_izq = get_distance_in_sector(-30, -5)
	distancia_abanico_der = get_distance_in_sector(5, 30)
	print("Distancia abanico izquierdo", distancia_abanico_izq, "Distancia abanico derecho", distancia_abanico_der)
	if distancia_abanico_izq <= WALL_FRONT_DIST or distancia_abanico_der <= WALL_FRONT_DIST:
		msg.linear.x = 0
		msg.linear.y = 0
		msg.angular.z = 0
		vel_pub.publish(msg)
		del msg
		return "obstacle"
	elif dist_error <= dist:
		return "arrived"
	else:		
		msg.linear.x = v
		msg.angular.z = w
		vel_pub.publish(msg)
		del msg
		return "forward"
	

def follow_right_hand_wall():
	alpha = find_wall_direction_der(70, 90)
	anguloDerechaDeseado = 0
	distanciaDerechaDeseado = 0.30

	distance_to_right = get_distance_in_sector(88, 91)
	distanciaDerecha = distance_to_right * np.cos(alpha)

	errorAngulo = anguloDerechaDeseado - alpha
	errorDistancia = distanciaDerechaDeseado - distanciaDerecha 

	print('alpha: ' + str(alpha) + ' distanciaDerecha: ' + str(distanciaDerecha) + ' error angulo: ' + str(errorAngulo) + ' error distancia: ' + str(errorDistancia))
	
	kp_alpha = 0.9
	kp_dist = 1
	if get_distance_in_sector(-3, 3) < 1.5:
		v = 0.05
		w = 0.3
	else:
		v = 0.1
		w = (kp_alpha * errorAngulo) + (kp_dist * errorDistancia)
	
		
	limVelocidad = 0.3
	#saturacion
	if w > limVelocidad:
		w = limVelocidad
	elif w < -limVelocidad:
		w = -limVelocidad
					
	msg = Twist()
	msg.angular.z = w
	msg.linear.x = v
	vel_pub.publish(msg)

	if abs(get_robot_angle()) <= np.deg2rad(2) and get_distance_in_sector(-30, 30) >= 1 or ((goal_x - robot_x) <= 0.05 and (goal_y - robot_y) <= 0.05):
		return True
	else:
		return False

def follow_left_hand():
	global t
	kp_alpha = 2
	kp_dist = 3

	distance_to_right = get_distance_in_sector(- 90, -86)
                                                           
	#--------------------------------------------------------------
	#--------------------------------------------------------------
	alpha = find_wall_direction_ziq()
	anguloDerechaDeseado = 0
	distanciaDerechaDeseado = -0.3
	#distanciaFrontalMaxima = 1
	
	#print("DISTANCIA A LA IZQUIERDA: ", distance_to_right)
	distanciaDerecha = distance_to_right * np.cos(alpha)

	errorAngulo = anguloDerechaDeseado + alpha
	errorDistancia = distanciaDerechaDeseado + distanciaDerecha

	print("Angulo", alpha, "Error angulo", errorAngulo)
	print("DistanciaActual", distanciaDerecha, "Error distancia", errorDistancia)

	kp_alpha = 3
	kp_dist = 2
	#print( self.scan.ranges[360])
	if get_distance_in_sector(-3, 3) < 1.5:
		print("FRENTE PARED ===")
		v = 0.05
		w = -0.3
	else:
		#pass
		v = 0.1
		w = (kp_alpha * errorAngulo) + (kp_dist * errorDistancia)
	#w = (kp_alpha * errorAngulo) + (kp_dist * errorDistancia)

	limVelocidad = 0.3
	#saturacion
	if w > limVelocidad:
		w = limVelocidad
	elif w < -limVelocidad:
		w = -limVelocidad
	
	#rospy.loginfo("{} {} {}".format(errorAngulo, errorDistancia, w))
					
	msg = Twist()
	msg.angular.z = w
	msg.linear.x = v
	#print(msg)
	vel_pub.publish(msg)

	if abs(get_robot_angle()) <= np.deg2rad(2) and get_distance_in_sector(-30, 30) >= 1 or ((goal_x - robot_x) <= 0.05 and (goal_y - robot_y) <= 0.05):
		return True
	else:
		return False


def main():
	global right_wall, front_wall, goal_y, goal_x, t0
	# se calcula la pendiente
	m = calculate_m()
	print("Slope: ", m)
	puntos = get_new_goal()
	side = None
	
	state = states[0]
	
	goal_x, goal_y = next(puntos)
	print(goal_x, goal_y)
	while not rospy.is_shutdown():

		if scan is not None and odom is not None:
			try:
				print("Point ({},{})\t Robot ({},{}, {})".format(goal_x, goal_y, robot_x, robot_y, robot_q))
				
				if state == "turn_to_goal":
					print("TURNING TO GOAL")
					status = turn_to_goal()
					if status:
						state = "go_to_goal"
				elif state == "go_to_goal":
					print("GOING TO GOAL")
					status = go_to_goal()
					if status == "obstacle":
						state = "select_side"
					elif status == "arrived":
						state = "get_goal"
				elif state == "select_side":
					side = side_check()
					if side == "left":
						state = "obstacle_left"
						#state = states[4]
					elif side == "right":
						state = "obstacle_right"
					t0 = rospy.Time.now().to_sec()
				elif state == "obstacle_right":
					print("Obstacle at right")
					status = follow_right_hand_wall()
					if status:
						state = "go_to_goal"
				elif state == "obstacle_left":
					print("Obstacle at left")
					status = follow_left_hand()
					if status:
						state = "go_to_goal"
				elif state == "get_goal":
					goal_x, goal_y = next(puntos)
					m = calculate_m()
					print("New Slope: ", m)
					state = "turn_to_goal"
				elif state == "stop":
					msg = Twist()
					msg.angular.z = 0.0
					msg.linear.x = 0.0
					msg.linear.y = 0.0
					vel_pub.publish(msg)
					rospy.loginfo("Stopping robot")
			except StopIteration:
				print("Completed map points")
				msg = Twist()
				msg.angular.z = 0.0
				msg.linear.x = 0.0
				msg.linear.y = 0.0
				vel_pub.publish(msg)
				#print("Restarting points")
				#puntos = get_new_goal()
				# getting a new goal
				#print("Arrived to goal")
				#print("Getting new goal")
				#goal_x, goal_y =  next(puntos)
				

		rate.sleep()
	


if __name__ == '__main__':
	try:
		rospy.init_node('navigation_node')
		scan_listener = rospy.Subscriber('/scan', LaserScan, scan_callback)
		odom_listener = rospy.Subscriber('/odom',  Odometry, odom_callback)
		vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		rate = rospy.Rate(10)
		main()
	except rospy.ROSInterruptException:
		print("Deteniendo")
		exit()
