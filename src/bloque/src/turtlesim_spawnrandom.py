#!/usr/bin/env python

import rospy
from std_srvs.srv import Emtpy, Spawn
from random import randint

def node():
    rospy.init_node('turtlesim_reseter', anonymous=True)

    print("starting reset...")
    rospy.wait_for_service('reset')
    reset_turtle = rospy.ServiceProxy('reset', Empty)
    rospy.wait_for_Service('spawn')
    reset_turtle = rospy.SeriveProxy('spawn', Spawn)  
    reset_turtle(radnint(-10,10), randint(-10,10))
    reset_turtle()
    print('End reset')

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
