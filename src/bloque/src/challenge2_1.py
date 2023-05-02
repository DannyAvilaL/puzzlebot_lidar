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
newT = 0
t0 = 0
mu = 0
sigma = 0.1
alpha_v = alpha_w = rd.gauss(mu,sigma)
mu_th = 0
#covarianza = [0.0 for x in range(36)]
covarianza = np.zeros((6,6), float)


sigmaM = np.ones((6,6), float)


qk = np.array([[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
			   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
			   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
			   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
			   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
			   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

#jacobiano

hq  = np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
    	        [0.0, 1.0, 0.0, 0.0, 0.0, 0.0], 
		        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0], 
		        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
		        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
		        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])


wl = wr = 0
a, b = hq[0][2], hq[1][2]
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
		file.writelines("{},{},{}\n".format(x, y, q))
	archivo = True

def odometry(covarianza):
# Odom ---------------------------------------------------
	odom = Odometry()
	#print("COVARIANZA: ", covarianza)
	odom.header.stamp = rospy.Time.now()
	odom.header.frame_id = "map"
	odom.child_frame_id = "base_link"
	odom.pose.pose.position.x = x
	odom.pose.pose.position.y = y
	odom.pose.pose.position.z = r
	odom.pose.pose.orientation.x = 0.0
	odom.pose.pose.orientation.y = 0.0
	odom.pose.pose.orientation.z = 0.0
	odom.pose.pose.orientation.w = 0.0
	odom.pose.covariance = covarianza.flatten().tolist()
	odom.twist.twist.linear.x = 0.0
	odom.twist.twist.linear.y = 0.0
	odom.twist.twist.linear.z = 0.0	
	odom.twist.twist.angular.x = 0.0
	odom.twist.twist.angular.y = 0.0
	odom.twist.twist.angular.z = q
	odom.twist.covariance = covarianza.flatten().tolist()
   	# publish the message#
	odom_pub.publish(odom)
	

def jacobian(newT, v, q): 
	"hq local en jacobian"
	global hq, a ,b 
	seno = np.sin(q)
	coseno = np.cos(q)
	a  += (-newT*v*seno)
	b += (newT*v*coseno)
	hq[0][2], hq[1][2] = a, b
	return hq, hq.transpose()

def covarianza():
	global x, y, q, wr, wl, newT, qk, sigmaM
	kr, kl = 0.8, 0.8
	cos = np.cos(q)
	sin = np.sin(q)
	sigma_delta_tiempo = np.array([np.round(kr*np.abs(wr),3),0.0,
											 0.0,            np.round(kl*np.abs(wl),3),
											 0.0,            0.0])
	matriz = np.array([cos,cos, 
			 sin, sin, 2/l, -2/l])
	mul = (r *newT)/2
	nabla_w = mul*matriz
	
	qk[0] = nabla_w * sigma_delta_tiempo * nabla_w.transpose()
	return qk

def sigma_cal (v, q):
	global newT, sigmaM
	hq, hqT = jacobian(newT, v, q)
	qk = covarianza()
	print("Covarianza", qk)
	sigmaM *= hq * hqT +  qk
	print("JACOBIANO : ", hq)
	print("Sigma", sigmaM)
	print("Resultado0", hq * hqT *  qk)
	

def node():
	global x, y, q, tiempo, archivo, alpha_w, alpha_v, wl, wr,t, newT, sigmaM, hq, qk
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
			sigmaM = np.ones((6,6), float)
			hq  = np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
    	        [0.0, 1.0, 0.0, 0.0, 0.0, 0.0], 
		        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0], 
		        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
		        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
		        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
			qk = np.array([[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
			   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
			   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
			   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
			   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
			   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
			print('Sigma reset---------------')
			print(sigmaM)	


			pWl.publish(0)
			pWr.publish(0)
			
		pCMvel.publish(vel)
		
		robotLocation.pose.position= Point(x,y,0)
		qRota = tf.transformations.quaternion_from_euler(0,0,q)
		robotLocation.pose.orientation = Quaternion(qRota[0],qRota[1],qRota[2],qRota[3])
		cTime = rospy.Time.now()

		robotLocation.header.stamp = cTime
		robotLocation.header.frame_id = "base_footprint"
		rospy.loginfo("Tiempo total: {}".format(tiempo))
		tiempo += T
		pPose.publish(robotLocation)
		js = JointState()
		js.name = ["right_wheel_joint", "left_wheel_joint"]
		#t = cTime - t0
		js.position = [wr*newT, wl*newT]
		js.header.stamp = cTime
		t = cTime - t0
		newT = t.to_sec()
		pJS.publish(js)
		tb.sendTransform([x,y,0], qRota, cTime, "base_link", "map")
		odometry(covarianza())
		rate.sleep()

if __name__ == '__main__':
	try:
		rospy.init_node('puzzlebot_kinematic_model_1')
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
		t = 0
		fs = 50
		rate = rospy.Rate(fs)

		node()
	except rospy.ROSInterruptException:
		pWl.publish(0)
		pWr.publish(0)
		print("Deteniendo")
		exit()


