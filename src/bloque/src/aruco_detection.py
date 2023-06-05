#!/usr/bin/env python
import rospy 
import cv2 
from cv2 import aruco
import numpy as np    
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
from std_msgs.msg import Int32 
from geometry_msgs.msg import Pose2D
import tf

bridge = CvBridge()
cv_image = None
dict_aruco = aruco.Dictionary_get(aruco.DICT_7X7_100)
#lista_arucos =[0, 1, 3, 4, 5, 6]

parameters = aruco.DetectorParameters_create()
parameters.adaptiveThreshWinSizeMin = 3
parameters.adaptiveThreshWinSizeMax = 9
parameters.adaptiveThreshWinSizeStep = 1
parameters.errorCorrectionRate = 0.1
parameters.minMarkerDistanceRate = 30  
parameters.polygonalApproxAccuracyRate = 0.04
#parameters.minOtsuStdDev = 2
#parameters.minMarkerPerimeterRate = 0.035
#parameters.minCornerDistanceRate = 0.035
mtx = np.array([[800.0,    0.0,  400.0],
                [0.0,    800.0,  400.0],
                [0.0,      0.0,    1.0]])

dist = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])

pose_aruco = Pose2D()

def image_callback(img):
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(img, desired_encoding='rgb8') #passthrough

def aruco_identify(img):
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    maker_corners, maker_ids, rejectedImgPoints = aruco.detectMarkers(gray, dict_aruco, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(img.copy(), maker_corners, maker_ids)
    return frame_markers, maker_corners, maker_ids


def main():
    marker_size = 0.33
    
    while not rospy.is_shutdown():
        rate.sleep()
        if cv_image is not None:
            frame_markers, maker_corners, maker_ids = aruco_identify(cv_image)
            if np.all(maker_ids != None):
                marcador = maker_ids[0][0]
                rvec, tvec,_ = aruco.estimatePoseSingleMarkers(maker_corners, 0.33, mtx, dist)
                #rvec, tvec,_ = cv2.solvePnP(marker_points, maker_corners, mtx, dist, False, cv2.SOLVEPNP_IPPE_SQUARE)
                print(rvec)
                magnitud = np.sqrt(tvec[0][0][0]**2 + tvec[0][0][-1]**2)######
                pose_aruco.x, pose_aruco.y, pose_aruco.theta = magnitud, 0, np.cos(rvec[0][0][-1]) #######

                for i in range(0, maker_ids.size):
                    frame_markers = aruco.drawAxis(frame_markers, mtx, dist, rvec[i], tvec[i], 0.3)
                    #cv2.drawFrameAxes(frame_markers, mtx, dist, rvec[i], tvec[i], 0.3)
                    output_image = bridge.cv2_to_imgmsg(frame_markers, encoding = 'rgb8') #passthrough
            else:
                marcador = -1 
                output_image = bridge.cv2_to_imgmsg(cv_image, encoding = 'rgb8')

            pose_aruco_pub.publish(pose_aruco)
            aruco_detected_pub.publish(marcador)
            output_image = bridge.cv2_to_imgmsg(frame_markers, encoding = 'rgb8') #passthrough	
            image_process_pub.publish(output_image)

if __name__ == '__main__':
    try:
        rospy.init_node("aruco_detection")
        rate = rospy.Rate(20)
        image_process_pub = rospy.Publisher("/camera/image_raw/detected_img", Image, queue_size=1)
        image_raw_sub = rospy.Subscriber("/camera/image_raw", Image, image_callback)
        aruco_detected_pub = rospy.Publisher("/aruco_detected", Int32, queue_size=1) 
        pose_aruco_pub = rospy.Publisher("/pose_aruco", Pose2D, queue_size=1)

        main()

    except rospy.ROSInterruptException:
        print("Deteniendo")
        exit()
    