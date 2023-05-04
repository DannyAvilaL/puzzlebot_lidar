#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

scan = None
wall_dist = 1
w_max = 1
v_max = 1

w = 0
v = 0.3


def scan_callback(msg):
	global scan
	scan = msg


def go_to_wall():
	msg = Twist()
	msg.linear.x = 0.2
	if scan.ranges[360] < wall_dist:
		msg.linear.x = 0
	
	vel_pub.publish(msg)

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
	
	start_index = range_index(start_angle)
	end_index = range_index(end_angle)
	sector_ranges = scan.ranges[end_index:start_index]
	return np.mean(sector_ranges)


def find_wall_direction():
	"""
	Asume que hay una pared en la derecha
	"""
	hip = get_distance_in_sector(np.radians(45-1), np.radians(45))
	ady = get_distance_in_sector(np.radians(90-1), np.radians(90))

	alpha = np.arctan2(hip*np.cos(np.deg2rad(45))-ady, hip*np.sin(np.deg2rad(45)))

	return alpha


def follow_right_hand_wall():
	found_wall = False
	distance_ahead = get_distance_in_sector(np.pi/4, -np.pi/4)



def main():
	while not rospy.is_shutdown():
		if scan is not None:
			pass

if __name__ == '__main__':
	try:
		rospy.init_node('bug0_node')
		scan_listener = rospy.Subsriber('/scan', LaserScan, scan_callback)
		vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		rate = rospy.Rate(10)

		main()
	except rospy.ROSInterruptException:
		
		print("Deteniendo")
		exit()


