#!/usr/bin/env python
""" Simple feedforward control that makes the puzzlebot move in a circle with given radius and linear velocity.
"""

import sys
import numpy as np
import rospy
from geometry_msgs.msg import Twist

class MoveInCircleCommander():
    def __init__(self):

        rospy.init_node('MoveInCircle')

        # Publish to the /cmd_vel topic
        self.pub = rospy.Publisher("/vel_cmd", Twist, queue_size=1)
        self.rate = rospy.Rate(20)
        

    def main(self, radie=1.2, linvel=0.1 ):

        # If there's an object attached to the ee we want it to follow its trajectory
        while not rospy.is_shutdown():
            self.rate.sleep()

            self._move_in_circle(radie, linvel)
                    
                    
    def _move_in_circle(self, radie, linvel):

        tw = Twist()
        
        #----------------------------------------------------------------------------
        # Your code here
        #----------------------------------------------------------------------------

        self.pub.publish(tw)
        

        
if __name__ == '__main__':

    try:
        node = MoveInCircleCommander()
        if len(sys.argv) >= 3:
            node.main(radie=sys.argv[1], linvel=sys.argv[2])
        else:
            node.main()
    except (rospy.ROSInterruptException, rospy.ROSException("topic was closed during publish()")):
        pass
