#!/usr/bin/env python

import tf, rospy
from sensor_msgs.msg import JointState
import numpy as np
from std_msgs.msg import Float32
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
import random as rd
from nav_msgs.msg import Odometry

pathFile = "data_record.txt"

x, y, q = 0.0, 0.0, 0.0 
r, l = 0.05, 0.188
v, w = 0.1, 0.0
tiempo = 0
archivo = False
t0 = 0
mu = 0
sigma = 0.1
alpha_v = alpha_w = rd.gauss(mu,sigma)
mu_th = 0
covarianza = [0.0 for x in range(36)]
qk = hq = sigmaM = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
t = 0

wl = wr = 0

def callback_top(msg):
    global v, w
    v = msg.linear.x
    w = msg.angular.z


def callback_ser(req):
    global x, y, q, tiempo, archivo
    x, y, q = 0.0, 0.0, 0.0
    tiempo = 0.0
    archivo = False
    return EmptyResponse()


def guardaArchivo():
	global archivo
	with open(pathFile, 'a') as file:
		file.writelines("{},{},{}\n".format(x, y,q))
	archivo = True

def odometry(covarianza):
# Odom ---------------------------------------------------
	odom = Odometry()
	print("COVARIANZA en 0", covarianza)
	odom.header.stamp = rospy.Time.now()
	odom.header.frame_id = "world"
	odom.child_frame_id = "base_link"
	odom.pose.pose.position.x = x
	odom.pose.pose.position.y = y
	odom.pose.pose.position.z = r
	odom.pose.pose.orientation.x = 0.0
	odom.pose.pose.orientation.y = 0.0
	odom.pose.pose.orientation.z = 0.0
	odom.pose.pose.orientation.w = 0.0
	odom.pose.covariance = covarianza[:]
	odom.twist.twist.linear.x = 0.0
	odom.twist.twist.linear.y = 0.0
	odom.twist.twist.linear.z = 0.0	
	odom.twist.twist.angular.x = 0.0
	odom.twist.twist.angular.y = 0.0
	odom.twist.twist.angular.z = q
	odom.twist.covariance = covarianza[:]
   	# publish the message#
	odom_pub.publish(odom)
	#print(odom)

def jacobian(tiempo, v, q): 
	"hq local en jacobian"
	global hq,t
	hq += np.array([
		  1.0, 0.0, np.round(-t*v*np.sin(q),3), 0.0, 0.0, 0.0, 
          0.0, 1.0, np.round(t*v*np.cos(q),3), 0.0, 0.0, 0.0, 
		  0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 
		  0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
		  0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
		  0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
	return hq, hq.transpose()

def covarianza():
	global x, y, q, wr, wl
	kr, kl = 1, 1
	cos = np.cos(q)
	sin = np.sin(q)
	sigma_delta_tiempo = np.array([np.round(kr*np.abs(wr),3), 0.0, 0.0, np.round(kl*np.abs(wl),3), 0.0, 0.0])
	print(sigma_delta_tiempo)
	delta_w = (1/2)*r*t*np.array([np.round(cos,3), np.round(cos,3), np.round(sin,3), np.round(sin,3), 2/l, -2/l])
	print(delta_w)
	qk[:6] = delta_w * sigma_delta_tiempo * delta_w.transpose()
	return qk

def sigma_cal (v, q):
	global tiempo, sigmaM
	hq, hqT = jacobian(tiempo, v, q)
	qk = covarianza()
	sigmaM *= hq * hqT *  qk
	return sigmaM

def node():
	global x, y, q, tiempo, archivo, alpha_w, alpha_v, wl, wr
	T = 1.0/fs # 0.02
	while not rospy.is_shutdown():

		robotLocation = PoseStamped()
		vel = Twist()
		

		if tiempo < 10.0:
			# -------------------------------------------------------
			# x += T*(v+alpha_v)*np.cos(q)
			# y += T*(v+alpha_v)*np.sin(q)
			# q += T*(w+alpha_w)
			x += T*v*np.cos(q)
			y += T*v*np.sin(q)
			q += T*w
			# -------------------------------------------------------
			wl = (v - 0.5*l*w)/r
			wr = (v + 0.5*l*w)/r
			# -------------------------------------------------------
			pWl.publish((v - 0.5*l*w)/r)
			pWr.publish((v + 0.5*l*w)/r)

			vel.linear.x = (wl+wr) / 20
			vel.angular.z = w

			sigma_cal(v, q)

			alpha_w = alpha_v = rd.gauss(0,0.1)
			
		else:
			if not archivo:
				archivo = True
				guardaArchivo()
				reset = rospy.ServiceProxy('/reset', Empty)
				reset()
				vel.linear.x = 0
				vel.angular.z = 0

			pWl.publish(0)
			pWr.publish(0)
			
		#pCMvel.publish(vel)
		
		robotLocation.pose.position= Point(x,y,0)
		qRota = tf.transformations.quaternion_from_euler(0,0,q)
		robotLocation.pose.orientation = Quaternion(qRota[0],qRota[1],qRota[2],qRota[3])
		cTime = rospy.Time.now()
		t = cTime - t0
		robotLocation.header.stamp = cTime
		robotLocation.header.frame_id = "base_footprint"
		rospy.loginfo("Tiempo total: {}".format(tiempo))
		tiempo += T
		pPose.publish(robotLocation)
		js = JointState()
		js.name = ["right_wheel_joint", "left_wheel_joint"]
		js.position = [wr*t.to_sec(), wl*t.to_sec()]
		js.header.stamp = cTime
		pJS.publish(js)
		tb.sendTransform([x,y,0], qRota, cTime, "base_link", "map")
		odometry(covarianza())
		rate.sleep()

if __name__ == '__main__':
	try:
		rospy.init_node('puzzlebot_kinematic_model')
		t0 = rospy.Time.now()
		ser = rospy.Service("/reset", Empty, callback_ser)
		tb = tf.TransformBroadcaster()
		pPose = rospy.Publisher('/pose', PoseStamped, queue_size=10)
		pWl   = rospy.Publisher('/wl', Float32, queue_size=10)
		pWr   = rospy.Publisher('/wr', Float32, queue_size=10)
		rospy.Subscriber("/cmd_vel", Twist, callback_top)
		pCMvel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		pJS = rospy.Publisher("/joint_states", JointState, queue_size=10)
		odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
		fs = 50
		rate = rospy.Rate(fs)

		node()
	except rospy.ROSInterruptException:
		pWl.publish(0)
		pWr.publish(0)
		print("Deteniendo")
		exit()


