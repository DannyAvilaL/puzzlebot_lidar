#!/usr/bin/env python

import math
from cv2 import aruco
from math import sin, cos, pi
import rospy
import tf
import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped, Pose2D, TransformStamped
import numpy as np
from std_msgs.msg import Float32, Int32
from std_srvs.srv import Empty, EmptyResponse
from fiducial_msgs.msg import FiducialTransformArray
import tf_conversions
#from geometry_msgs.msg import TransformStamped, Twist


rospy.init_node('odometry_publisher')

# ========= PUBLICADORES A TOPICOS =================
odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)
pJS = rospy.Publisher("/joint_states", JointState, queue_size=10)
pPose = rospy.Publisher('/pose', PoseStamped, queue_size=10)


# =========== OBJETOS DE POSICION =============
odom_broadcaster = tf.TransformBroadcaster() 
tb = tf.TransformBroadcaster() # Aruco
vel = Twist()
robotLocation = PoseStamped()

# ========== PARA LOS ARUCOS (TF o TF2) ===========
ta = tf2_ros.TransformBroadcaster() 
#tfBuffer = tf2_ros.Buffer(rospy.Time(30))
#transformListener = tf.TransformListener(tfBuffer) 

# ============ VARIABLES INICIALES ===========
pathFile = "./data_record.txt"
x = 0.0
y = 0.0
th = 0.0
t0 = 0

r, l = 0.05, 0.188
v, w = 0.0, 0.0
wr = wl = 0.0

current_time = rospy.Time.now()
last_time = rospy.Time.now()

newT = 0
rate = rospy.Rate(10)

# ======== ARUCOS =========
M = {111: [2, -1], 101: [5, -2], 102: [9, 1], 100: [3, 4], 103:[8,-2], 109:[8,2]}
medicion_ArUco, ArUco_id = Pose2D(), -1
aruco_id = None
trans, rot = 0, 0
magnitud,angulo =0,0

# =========== MATRICES DE APOYO ===========
mu = np.array([x, y, th]).T

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

# ========== SUSCRIPTORES ==========0
def callback_wl(msg):
    global wl
    wl = msg.data

def callback_wr(msg):
    global wr
    wr = msg.data

def callback_ArUco(msg):
    global ArUco_id
    ArUco_id = msg.data

def callback_medicion_ArUco(msg):
    global medicion_ArUco
    medicion_ArUco.x = msg.x
    medicion_ArUco.y = msg.y
    medicion_ArUco.theta = msg.theta

def callback_fiducials(msg):
    global aruco_id, trans, rot, magnitud, angulo
    image_time = msg.header.stamp
    found = False
    
    if len(msg.transforms) == 0:
        aruco_id = -1
        return
    #por cada aruco, se publica su transformada
    for aruco in msg.transforms:
        # if the aruco is of interest
        if aruco.fiducial_id in M.keys():
            print("SI EXISTE EL ARUCO\n\n")
            aruco_id = aruco.fiducial_id
            trans = aruco.transform.translation
            rot = aruco.transform.rotation
            x = trans.z
            y = trans.x * -1
            z = trans.y * -1
            magnitud = np.sqrt(x**2 + y**2)#np.sqrt(trans.x**2 + trans.y**2 + trans.z**2)
            #angulo = rot.y * -1
            angulo = np.arctan2(y, x)
            #print("Trans X", trans.x, type(trans.x))

            t = TransformStamped()
            t.header.frame_id = msg.header.frame_id  # msg.header.frame_id  to base_link
            t.child_frame_id = "f_id_" + str(id) # camara 
            t.header.stamp = image_time
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            #q = tf_conversions.transformations.quaternion_from_euler(rot.x, rot.y, rot.z)
            t.transform.rotation.x = rot.x
            t.transform.rotation.y = rot.y
            t.transform.rotation.z = rot.z
            t.transform.rotation.w = rot.w
            ta.sendTransform(t) 

            #aruco_q = tf.transformations.quaternion_from_euler(rot.x, rot.y, rot.z)
            #tr = tfBuffer.lookup_transform("base_link", t.child_frame_id, image_time) # tf2
            #ct = tr.transform.translation
            #cr = tr.transform.rotation

            #print("T_fidBase %lf %lf %lf %lf %lf %lf %lf\n" % \
            #                     (ct.x, ct.y, ct.z, cr.x, cr.y, cr.z, cr.w))
        else:
            print("NO EXISTE EL ARUCO\n\n")
            magnitud, angulo = 0,0
            aruco_id = -1
        return 



rospy.Subscriber("/wl", Float32, callback_wl)
rospy.Subscriber("/wr", Float32, callback_wr)
rospy.Subscriber('/aruco_detected', Int32, callback_ArUco)
#rospy.Subscriber('/pose_aruco', Pose2D, callback_medicion_ArUco)
rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, callback_fiducials)


def EKF(Muk_ant, sigmak_ant, Zik, Rk):
    global ArUco_id, aruco_id
    '''
    Inputs
        -Muk_ant: vector mu obtenido
        -sigmak_ant: matriz sigma obtenida
        -Zik: medicion
        -Rk: Covarianza para la medicion
    Outputs
        -Muk
        -sigmak
    '''
    Zik = np.array(Zik)
    Muk = Muk_ant 
    sigmak = sigmak_ant

    #if (aruco_id != None):###############si ve aruco, mi - coordenada
    if aruco_id in M.keys():
        delta_x = M[aruco_id][0] - Muk[0]
        delta_y = M[aruco_id][1] - Muk[1]
        p = delta_x**2 + delta_y**2
        Gk = np.array([[-delta_x/(p**0.5), -delta_y/(p**0.5), 0],
                       [delta_y/p,         -delta_x/p,       -1]])
        
        Zik_e = np.array([p**0.5,
                 np.arctan2(delta_y, delta_x) - Muk[2]]) 
        
        Zk = np.dot(Gk, np.dot(sigmak, Gk.T)) + Rk

        Kk = np.dot(sigmak, np.dot(Gk.T, np.linalg.inv(Zk)))
        #resta en ecuacion de Manchester: Zik - Zik_e
        Muk += np.dot(Kk, (Zik - Zik_e))
        #print("Diferencias Zk y Zke", Zik - Zik_e, Zik)
        print("Diferencias Zk y Zke----------------------------")
        print('zik: ',  Zik)
        print('zik_e: ',  Zik_e)
        print('diferencia: ', (Zik - Zik_e))

        I = np.eye(3) 
        sigmak = np.dot((I - np.dot(Kk, Gk)), sigmak)

    return Muk, sigmak

def calculoQK():
    global QK
    # matriz de relacion entre cambio de posicion y velocidad angular
    dif_w = 0.5 * r * dt * np.array([[np.cos(mu[2]), np.cos(mu[2])],
                        [np.sin(mu[2]), np.sin(mu[2])],
                        [2/l, -2/l]])
    dif_wT = dif_w.transpose()
    
    kr, kl = 1, 1
    varianza = [[kr*abs(wr), 0],
                [0, kl * abs(wl)]]

    QK = np.dot(dif_w, np.dot(varianza, dif_wT))


def propagacionError(dt, v, th):
    global sigma
    #Jacobiano
    H = np.array([  [1.0, 0.0, -dt * v * np.sin(th)],
                    [0.0, 1.0, dt * v * np.cos(th)],
                    [0.0, 0.0, 1.0]])
    
    HT = H.transpose()
    calculoQK()
    sigma = np.dot(np.dot(H,sigma),HT) + QK
    

last_time = rospy.Time.now()
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
            th += (dt * w)

            
            if th >= np.pi:
                th = -pi
                mu[2] = -pi
            elif th <= -np.pi:
                th = pi
                mu[2] = pi

            # para joints de llantas
            # wl = (v - 0.5*l*w)/r
            # wr = (v + 0.5*l*w)/r
            # cov = sigma.flatten().tolist()
            Zik = [magnitud, angulo] 
            #Zik = tf.lookupTransform()
            error = 0.1
            Rk = np.eye((2)) * error
            mu, sigma = EKF(mu, sigma, Zik, Rk)
            sig = sigma.flatten().tolist()

            x, y, odom_quat = mu[0], mu[1], tf.transformations.quaternion_from_euler(0, 0, mu[2])
            vel.linear.x = v

            vel.angular.z = w
            # first, we'll publish the transform over tf
            odom_broadcaster.sendTransform(
                (x, y, 0.),
                odom_quat,
                current_time,
                "odom",
                "map"
            )


            propagacionError(dt, v, mu[2])

            odom = Odometry()
            odom.header.stamp = current_time
            # odom.header.frame_id = "base_footprint"
            odom.header.frame_id = "map"
            odom.child_frame_id = "base_link"
            cov = np.zeros(36) #+ sigma.flatten().tolist()

            
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
            print(Vector3(mu[0], mu[1], mu[2]))
            odom.twist.twist = Twist(Vector3(v, 0 , 0), Vector3(0, 0, w))

            odom_pub.publish(odom)

            cTime = rospy.Time.now()
            
            t = cTime.to_sec() - t0
            robotLocation.pose.position= Point(mu[0],mu[1],r)
            qRota = tf.transformations.quaternion_from_euler(0,0,mu[2])
            robotLocation.pose.orientation = Quaternion(qRota[0],qRota[1],qRota[2],qRota[3])
            robotLocation.header.stamp = cTime
            robotLocation.header.frame_id = "base_link"
            pPose.publish(robotLocation)
            tb.sendTransform([mu[0],mu[1],0], qRota, cTime, "base_link", "map")
           
            js = JointState()
            js.name = ["base_to_right_w", "base_to_left_w"]
            js.position = [wr*t, wl*t]
            js.header.stamp = cTime
            pJS.publish(js)
            tb.sendTransform([x,y,0], qRota, cTime, "base_link", "map")
            last_time = current_time

            rate.sleep()


    except rospy.ROSInterruptException:
        vel.linear.x = 0
        vel.linear.y = 0
        vel.angular.z = 0
        cmdVel.publish(vel)
        print("Deteniendo")
        exit()