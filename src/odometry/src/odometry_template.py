#!/usr/bin/env python
""" Implement a node that calculates the odometry for the puzzlebot robot based on the angular velocities of the wheels
"""

import sys
import numpy as np
import rospy
import tf2_ros
from tf import transformations as trf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, TransformStamped, PoseWithCovariance,TwistWithCovariance
from std_msgs.msg import Float32

WHEEL_RADIUS = 0.05
WHEEL_DISTANCE = 0.18

class MyOdometryPublisher():
    def __init__(self):

        rospy.init_node('OdometryPublisher')

        # Subscribe to the wheel velocity topics
        rospy.Subscriber("/wl", Float32, self._wl_callback)
        rospy.Subscriber("/wr", Float32, self._wr_callback)
        self.wl = None
        self.wr = None
        
        # Publish to the odometry topic
        self.odom_pub = rospy.Publisher("/odometry", Odometry, queue_size=1)

        # Publish the simpler (estimated) state to separate topics
        self.x_pub = rospy.Publisher("/est_state/x", Float64, queue_size=1)
        self.y_pub = rospy.Publisher("/est_state/y", Float64, queue_size=1)
        self.th_pub = rospy.Publisher("/est_state/theta", Float64, queue_size=1)

        self.initial_state = np.array([0.0, 0.0, 0.0])
        self.model_state = None
        self.model_twist = None

        # Keep track of time between updates to the state of the robot
        self.current_time = rospy.get_time()
        self.last_time = current_time
        
        # For broadcasting transform from base_link to odom 
        self.br = tf2_ros.TransformBroadcaster() 

        self.rate = rospy.Rate(20)
        
    def _wl_callback(self, data):
        self.wl = data.data
    def _wr_callback(self, data):
        self.wr = data.data
        
        
    def main(self):

        while not rospy.is_shutdown():
            self.rate.sleep()

            if self.wl is not None and self.wr is not None:
                self.current_time = rospy.time()
                dt  = self.current_time - self.last_time

                #----------------------------------------------------------------------------
                # Your code here
                #
                # 1) Calculate the angular displacement of each wheel
                # 2) Calculate the linear- and angular displacement of the robot, using the function
                #    wheel_displacement_to_robot_displacement
                # 3) Calculate the new state of the robot
                # 4) Calculate the transform between the odometry frame and the base_link frame
                # 5) Calculate the twist of the robot
                # 6) Publish the results
                #----------------------------------------------------------------------------

                # Publish the transform between the odometry frame (fixed and coinciding with the base_frame at time=0)
                # and the base_link frame. It is the rotation and translation of the base_link frame with respect to the
                # odometry frame. It is at the same time the transform that takes a vector or point expressed in the base_link frame
                # and expresses it in the odometry frame.

                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "odom"
                t.child_frame_id = "base_link"
                #t.transform.translation = ?
                #t.transform.rotation =
                self.br.sendTransform(t)


                # Publish the odometry message
                odom = Odometry()
                # Fill the message with your data
                odom.header.stamp = rospy.Time.now()
                odom.header.frame_id = "odom"
                # Set the position
                odom.pose.pose = Pose(t.transform.translation, t.transform.rotation)
                # Set the velocity
                # odom.twist.twist = ?

                # publish the message
                self.odom_pub.publish(odom)


            # Publish the state of the wheeled robot to separate topics
            self.x_pub.publish(x_est)
            self.y_pub.publish(y_est)
            self.th_pub.publish(th_est)
                    



def wheel_displacement_to_robot_displacement(dL, dR, r=WHEEL_RADIUS, d=WHEEL_DISTANCE):
    """
    Returns the linear and angular displacement of the robot given the angular displacements of each wheel.
    Arguments
    ---------
       dL  :  float
         The angular displacement of the left wheel in rad.
       dR  :  float
         The angular displacement of the right wheel in rad.


    Returns
    --------
       d  :  float
            The linear displacement of the robot in m.
       dtheta  :  float
            The angular displacement of the robot in rad.
    Tests
    ------
    >>> # No displacement of left wheel, and 2pi rad displacement of right wheel gives pi r displacement of robot
    >>> d, dtheta = wheel_displacement_to_robot_displacement(0, 2*np.pi)
    >>> np.allclose(d, np.pi*WHEEL_RADIUS)
    True
    >>> np.allclose(dtheta, np.pi*WHEEL_RADIUS / WHEEL_DISTANCE)
    """
    #----------------------------------------------------------------------------
    # Your code here
    #
    # Calculate the linear and angular displacement of the robot given the angular displacements of each wheel.
    #
    #----------------------------------------------------------------------------
    return (d, dtheta)



if __name__ == '__main__':
    if len(sys.argv) > 1:
        if sys.argv[1] == "--test":
            import doctest
            doctest.testmod()
            sys.exit(0)
    try:
        aux = MyOdometryPublisher()
        aux.main()

    except (rospy.ROSInterruptException, rospy.ROSException("topic was closed during publish()")):
        pass
