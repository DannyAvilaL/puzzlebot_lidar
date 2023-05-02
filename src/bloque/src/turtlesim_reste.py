#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty

def node():
    rospy.init_node('turtlesim_reseter', anonymous=True)

    print("starting reset...")
    rospy.wait_for_service('reset')
    reset_turtle = rospy.ServiceProxy('reset', Empty)
    reset_turtle()
    print('End reset')

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
