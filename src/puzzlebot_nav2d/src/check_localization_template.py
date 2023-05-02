#!/usr/bin/env python
"""
Implement a node that checks the error in the localization against the known transform
between the initial pose of the robot and the map (same as gazebo).
"""

import numpy as np
import rospy
import tf2_ros
from tf import transformations as trf
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32

class LocalizationComparator():
    def __init__(self):

        rospy.init_node('LocalizationComparator')

        
        # Publish the errors
        self.dist_pub = rospy.Publisher("/odometry_error/distance", Float32, queue_size=1)
        self.ang_pub = rospy.Publisher("/odometry_error/angle", Float32, queue_size=1)

        self.rate = rospy.Rate(20)

        self.true_odom = None
        self.odom = None
        
    def _true_odom_callback(self, data):
        self.true_odom = data
    def _odom_callback(self, data):
        self.odom = data
        
        
    def main(self):


        while not rospy.is_shutdown():
            self.rate.sleep()

            if self.odom is not None and self.true_odom is not None:
                
                pos0 = self.true_odom.pose.pose.position
                orient0 = self.true_odom.pose.pose.orientation
                pos1 = self.odom.pose.pose.position
                orient1 = self.odom.pose.pose.orientation
                
                p0 = np.array([pos0.x, pos0.y, pos0.z])
                q0 = [orient0.x, orient0.y, orient0.z, orient0.w]
                p1 = np.array([pos1.x, pos1.y, pos1.z])
                q1 = [ orient1.x, orient1.y, orient1.z, orient1.w]

                q01 = trf.quaternion_multiply(trf.quaternion_conjugate(q0), q1)
                dist = np.linalg.norm(p1 - p0) # Both are in the same static reference system 
                angle = 2*np.arcsin(np.linalg.norm(q01[:3]))

                self.dist_pub.publish(dist)
                self.ang_pub.publish(angle)
                
if __name__ == '__main__':

    try:
        aux = OdometryComparator()
        aux.main()

    except (rospy.ROSInterruptException, rospy.ROSException("topic was closed during publish()")):
        pass
