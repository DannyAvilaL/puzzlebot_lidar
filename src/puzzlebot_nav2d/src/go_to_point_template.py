#!/usr/bin/env python  
import sys
import rospy
import math
import tf2_ros
import geometry_msgs.msg
from tf2_geometry_msgs import PointStamped

class Go2PointController:
    def __init__(self, v0=0.4, Kth=2, alpha=4 ):
        """
        Arguments
        --------------
        v0     :   Max linear velocity
        Kth    :   Gain for angular feedback controller
        alpha :  Parameter in model for controlling linear velocity as fcn of distance to target
        """
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.vel_pub = rospy.Publisher('/cmd_vel' , geometry_msgs.msg.Twist, queue_size=1 )
        self.rate = rospy.Rate(10.0)

        self.v0 = v0
        self.Kth = Kth
        self.alpha = alpha

    def go_to_point(self, x, y, frame):
        goal = PointStamped()
        goal.header.frame_id = frame
        goal.point.x = x
        goal.point.y = y
        goal.point.z = 0
        print(goal)
        
        while not rospy.is_shutdown():
            try:
                #goal.header.stamp = rospy.Time.now()
                # Should also work using
                 goal.header.stamp = rospy.Time(0)
                
                # Obtain the goal point expressed in the reference frame of base_link
                bot_goal = self.tf_buffer.transform(goal, 'base_link', timeout = rospy.Duration(1))

                #--------------------------------------------------------------
                # Your code here
                # 1) Calculate the errors
                # 2) Calculate the linear and angular velocities.
                # 3) Publish the twist message to /cmd_vel

                e_theta = 0.0
                e_dist = 0.0
                v = 0.0
                w = 0.0
                
                msg = geometry_msgs.msg.Twist()
                msg.angular.z = w
                msg.linear.x = v 
                print(msg)
                self.vel_pub.publish(msg)

                #--------------------------------------------------------------
                #--------------------------------------------------------------

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                #pass
                continue

        self.rate.sleep()        

if __name__ == '__main__':
    rospy.init_node('go_to_point_controller')
    g2p = Go2PointController()
    g2p.go_to_point(float(sys.argv[1]), float(sys.argv[2]), sys.argv[3])
    


