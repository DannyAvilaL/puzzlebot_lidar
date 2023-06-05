#!/usr/bin/env python

import math
from math import sin, cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
import numpy as np
from std_msgs.msg import Float32
from std_srvs.srv import Empty, EmptyResponse

rospy.init_node('odometry_publisher')

# ========= PUBLICADORES A TOPICOS =================
odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)
cmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
pJS = rospy.Publisher("/joint_states", JointState, queue_size=10)
pPose = rospy.Publisher('/pose', PoseStamped, queue_size=10)


# =========== OBJETOS DE POSICION =============
odom_broadcaster = tf.TransformBroadcaster()
tb = tf.TransformBroadcaster()
vel = Twist()
robotLocation = PoseStamped()

# ============ VARIABLES INICIALES ===========
pathFile = "./data_record.txt"
x = 0.0
y = 0.0
th = 0.0
t0 = 0
# vx = 0.1
# vy = 0.0
# vth = 0.0
r, l = 0.05, 0.188
v, w = 0.0, 0.0
current_time = rospy.Time.now()
last_time = rospy.Time.now()
newT, mu = 0, 0
rate = rospy.Rate(10)
wr = wl = 0.0
# =========== MATRICES DE APOYO ===========
mu = np.array([x, y, th])

sigma = np.array([[0.0, 0.0, 0.0],
         [0.0, 0.0, 0.0],
         [0.0, 0.0, 0.0]])

# Valores propuestos
QK = np.array([[0.5, 0.01, 0.01],
      [0.01, 0.5, 0.01],
      [0.01, 0.01, 0.2]])


# ========== SERVICIOS  ============
def callback_ser(req):
    global x, y, q, tiempo, archivo
    x, y, q = 0.0, 0.0, 0.0
    tiempo = 0.0
    archivo = False
    return EmptyResponse()

ser = rospy.Service("/reset", Empty, callback_ser)

# ========== SUSCRIPTIORES ==========0
def callback_wl(msg):
    global wl
    wl = msg.data

def callback_wr(msg):
    global wr
    wr = msg.data


rospy.Subscriber("/wl", Float32, callback_wl)
rospy.Subscriber("/wr", Float32, callback_wr)

def calculoQK():
    global QK
    # matriz de relacion entre cambio de posicion y velocidad angular
    dif_w = 0.5 * r * dt * np.array([[np.cos(mu[2]), np.cos(mu[2])],
                        [np.sin(mu[2]), np.sin(mu[2])],
                        [2/l, -2/l]])
    dif_wT = dif_w.transpose()
    
    kr, kl = 1, 1
    varianza = [[kr*wr, 0],
                [0, kl * wl]]

    QK = np.dot(dif_w, np.dot(varianza, dif_wT))


def propagacionError(dt, v, th):
    global sigma
    #Jacobiano
    H = np.array([[1.0, 0.0, -dt * v * np.sin(th)],
         [0.0, 1.0, dt * v * np.cos(th)],
         [0.0, 0.0, 1.0]])
    
    HT = H.transpose()
    calculoQK()
    sigma = np.dot(np.dot(H,sigma),HT) + QK
    #sigma = (H * sigma * HT) + QK

last_time = rospy.Time().now()
# ============ FLUJO PRINCIPAL ============
if __name__ == "__main__":
    try:
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - last_time).to_sec()

            # delta_x = (vx * cos(th) - vy * sin(th)) * dt
            # delta_y = (vx * sin(th) + vy * cos(th)) * dt
            # delta_th = vth * dt
            v = r*(wl + wr)/2
            w = r*(wr - wl)/ l
            mu[0] += dt * v * np.cos(th)
            mu[1] += dt * v * np.sin(th)
            mu[2] += dt * w

            x += dt * v * np.cos(th)
            y += dt * v * np.sin(th)
            th += dt * w

            # para joints de llantas
            # wl = (v - 0.5*l*w)/r
            # wr = (v + 0.5*l*w)/r

            odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
            vel.linear.x = v

            vel.angular.z = w
            # first, we'll publish the transform over tf
            odom_broadcaster.sendTransform(
                (x, y, 0.),
                odom_quat,
                current_time,
                "odom",
                "world"
            )


            propagacionError(dt, v, th)

            odom = Odometry()
            odom.header.stamp = current_time
            # odom.header.frame_id = "base_footprint"
            odom.header.frame_id = "world"
            odom.child_frame_id = "base_link"
            cov = np.zeros(36) #+ sigma.flatten().tolist()
            # cov = sigma.flatten().tolist()
            sig = sigma.flatten().tolist()
            print("SIG", sig)
            odom.pose.covariance[0]  = sig[0]
            odom.pose.covariance[1]  = sig[1]
            odom.pose.covariance[5]  = sig[2]
            odom.pose.covariance[6]  = sig[3]
            odom.pose.covariance[7]  = sig[4]
            odom.pose.covariance[11]  = sig[5]
            odom.pose.covariance[30]  = sig[6]
            odom.pose.covariance[31]  = sig[7]
            odom.pose.covariance[35]  = sig[8]
            print(odom.pose.covariance)

            # posicion
            odom.pose.pose = Pose(Point(x, y, r), Quaternion(*odom_quat))
            # velocidad
            # odom.child_frame_id = "base_link"
            print(Vector3(mu[0], mu[1], 0))
            odom.twist.twist = Twist(Vector3(mu[0], mu[1], r), Vector3(0, 0, mu[2]))

            odom_pub.publish(odom)

            cTime = rospy.Time.now()
            
            t = cTime.to_sec() - t0
            robotLocation.pose.position= Point(x,y,r)
            qRota = tf.transformations.quaternion_from_euler(0,0,th)
            robotLocation.pose.orientation = Quaternion(qRota[0],qRota[1],qRota[2],qRota[3])
            robotLocation.header.stamp = cTime
            robotLocation.header.frame_id = "base_link"
            pPose.publish(robotLocation)
            tb.sendTransform([x,y,0], qRota, cTime, "base_link", "world")
           
            js = JointState()
            js.name = ["base_to_right_w", "base_to_left_w"]
            js.position = [wr*t, wl*t]
            js.header.stamp = cTime
            pJS.publish(js)
            tb.sendTransform([x,y,0], qRota, cTime, "base_link", "world")
            last_time = current_time


            #cmdVel.publish(vel)


            rate.sleep()


    except rospy.ROSInterruptException:
        vel.linear.x = 0
        vel.linear.y = 0
        vel.angular.z = 0
        cmdVel.publish(vel)
        print("Deteniendo")
        exit()
