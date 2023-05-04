#!/usr/bin/env python
""" Implements a "ground truth" odometry using the model state information from Gazebo.
"""

import numpy as np
import rospy
import tf
from tf import transformations as trf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, TransformStamped, PoseWithCovariance, TwistWithCovariance, Point, Quaternion, Pose, Twist
from std_msgs.msg import Float32, Float64


class MyOdometryPublisher():
    def __init__(self):

        rospy.init_node('OdometryPublisher')

        # Subscribe to the wheel velocity topics
        rospy.Subscriber("/wl", Float32, self._wl_callback)
        rospy.Subscriber("/wr", Float32, self._wr_callback)
        self.wl = 0
        self.wr = 0
        self.th = 0

        # Robot geometry values
        self.l = 0.18
        self.wheel_r = 0.05

        # Publish to the odometry topic
        self.odom_pub = rospy.Publisher("/true_odometry", Odometry, queue_size=1)

        # Publish the simpler (estimated) state to separate topics
        self.x_pub = rospy.Publisher("/est_state/x", Float64, queue_size=1)
        self.y_pub = rospy.Publisher("/est_state/y", Float64, queue_size=1)
        self.th_pub = rospy.Publisher("/est_state/theta", Float64, queue_size=1)

        self.initial_state = np.array([0.0, 0.0, 0.0])
        self.pose = np.zeros(3) #[0.0, 0.0, 0.0]
        self.model_state = None
        self.model_twist = None

        # Keep track of time between updates to the state of the robot
        self.current_time = rospy.get_time()
        self.last_time = self.current_time
        
        # For broadcasting transform from base_link to odom 
        self.br = tf.TransformBroadcaster()

        self.rate = rospy.Rate(20)
        
    def _wl_callback(self, data):
        self.wl = data.data
    def _wr_callback(self, data):
        self.wr = data.data
        
        
    def main(self):
        x = 0
        y = 0
        th = 0

        # If there's an object attached to the ee we want it to follow its trajectory
        while not rospy.is_shutdown():
            self.rate.sleep()
            #---------------------------------------------------------------------------
            # Set the velocity
            v = (self.wr + self.wl) / 2
            w = (self.wl - self.wr) / self.l
            vx = v*np.cos(th)
            vy = v*np.sin(th)

            # Calculate the new pose
            dt  = (self.current_time - self.last_time)
            self.last_time = self.current_time
            x += (v * np.cos(self.pose[0])) * dt
            y += (v * np.sin(self.pose[1])) * dt
            th += w * dt
            
            # Update the pose state
            self.pose[0] = x
            self.pose[1] = y
            self.pose[2] = th

            odom_quat = trf.quaternion_from_euler(0, 0, th)
            # Calculate the pose covariance
            #
            #----------------------------------------------------------------------------

            # Publish the transform between the odometry frame (fixed) and the base_link frame

            # t = TransformStamped()
            # t.header.stamp = rospy.Time.now()
            # t.header.frame_id = "world"
            # t.child_frame_id = "base_link"
            # t.transform.translation.x = x
            # t.transform.translation.y = y
            # t.transform.translation.z = 0.0
            # t.transform.rotation = odom_quat
            # self.br.sendTransform(t)
            cTime = rospy.Time.now()
            self.br.sendTransform([x,y,0], odom_quat, cTime, "base_link", "map")

            # Publish the odometry message
            odom = Odometry()
            # Fill the message with your data
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "map"
            odom.child_frame_id = "base_link"
            # Set the position
            #pose_cov = np.cov(np.array([x, y, 0.0]), np.array([0.0, 0.0, th]))
            #rospy.loginfo("Len de pose_cov {}".format(len(pose_cov)))
            #t.transform.translation, t.transform.rotation
            #rospy.loginfo("Translation: {}".format(trf.translation))
            #rospy.loginfo("Rotation: {}".format(trf.transform.rotation))
            #odom.pose.pose = Pose(Point(x, y, 0.0), odom_quat)
            #odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0,0,w))
            odom.pose.pose = Pose(Point(x, y, 0.), 
                                  Quaternion(*odom_quat)
                                  )
            # = PoseWithCovariance(Pose(
            #                                    Point(x, y, 0.), 
            #                                    Quaternion(*odom_quat)
            #                                    ))

            #odom_cov = np.cov(np.array([vx, vy, 0.0]), np.array([0.0, 0.0, w]))
            odom.twist.twist = Twist( #TwistWithCovariance(Twist(
                                            Vector3(vx, vy, 0), 
                                            Vector3(0, 0, w)
                                                       )

            # publish the message
            self.odom_pub.publish(odom)


            # Publish the state
            self.x_pub.publish(x)
            self.y_pub.publish(y)
            self.th_pub.publish(th)
            self.current_time = rospy.get_time()
                    
if __name__ == '__main__':

    try:
        aux = MyOdometryPublisher()
        aux.main()

    except (rospy.ROSInterruptException, rospy.ROSException("topic was closed during publish()")):
        pass
