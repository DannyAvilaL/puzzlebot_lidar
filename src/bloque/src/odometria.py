#!/usr/bin/env python
""" Implements a "ground truth" odometry using the model state information
from Gazebo.
"""
import numpy as np
import rospy
import tf2_ros
from tf import transformations as trf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, TransformStamped, Point, Quaternion, Pose, Twist
from std_msgs.msg import Float32, Float64

class MyOdometryPublisher():
    def __init__(self):
        rospy.init_node('OdometryPublisher')
        self.rate = rospy.Rate(10)
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
        self.odom_pub = rospy.Publisher("/odometry", Odometry, queue_size=1)
        # Publish the simpler (estimated) state to separate topics
        self.x_pub = rospy.Publisher("/est_state/x", Float64, queue_size=1)
        self.y_pub = rospy.Publisher("/est_state/y", Float64, queue_size=1)
        self.th_pub = rospy.Publisher("/est_state/theta", Float64, queue_size=1)
        self.pose = np.array([0.0, 0.0, 0.0])
        # Keep track of time between updates to the state of the robot
        self.current_time = rospy.get_time()
        self.last_time = self.current_time

        # For broadcasting transform from base_link to odom
        self.br = tf2_ros.TransformBroadcaster() 


    def _wl_callback(self, data):
        self.wl = data.data
    def _wr_callback(self, data):
        self.wr = data.data


    def main2(self):

        # If there's an object attached to the ee we want it to follow its trajectory
        while not rospy.is_shutdown():
            self.rate.sleep()

            yk = self.pose
            th = self.pose[2]
            # Set the velocity
            v = self.wheel_r * ((self.wr + self.wl) / 2)
            w = self.wheel_r * ((self.wr - self.wl) / self.l)
            vx = v*np.cos(th)
            vy = v*np.sin(th)
            # Calculate the new pose
            dt = (self.current_time - self.last_time)
            self.last_time = self.current_time
            yk1 = yk + (np.array([vx, vy, w])) * dt
            # If the angle completes one full rotation, restart angle value
            if yk[2] >= 2*np.pi or yk[2] <= -2*np.pi:
                yk[2] = 0.0

            # Update the pose state
            self.pose = yk1
            odom_quat = trf.quaternion_from_euler(0, 0, yk[2])

            # Publish the transform between the odometry frame (fixed) and the base_link frame
            
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = yk[0]
            t.transform.translation.y = yk[1]
            t.transform.translation.z = 0.0
            t.transform.rotation = odom_quat
            self.br.sendTransform(t)
            # Fill the message with your data
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"

            # Set the position
            odom.pose.pose = Pose(Point(t.transform.translation.x,
            t.transform.translation.y, 0.0), Quaternion(*odom_quat))
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.angular.z = w
            # Publish the odometry message
            self.odom_pub.publish(odom)
            # Publish the state
            self.x_pub.publish(self.pose[0])
            self.y_pub.publish(self.pose[1])
            self.th_pub.publish(self.pose[2])
            self.current_time = rospy.get_time()
            rospy.loginfo("Translation: {}".format(t.transform.translation))
            rospy.loginfo("Rotation: {}".format(t.transform.rotation))

if __name__ == '__main__':
    try:
        aux = MyOdometryPublisher()
        aux.main2()
    except (rospy.ROSInterruptException, rospy.ROSException("topic was closed during publish()")):
        pass
