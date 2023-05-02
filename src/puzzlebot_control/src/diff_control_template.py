#!/usr/bin/env python  
import sys
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

WHEEL_DISTANCE = 0.18 # From puzzlebot_gazebo/urdf/puzzle_bot.xacro
WHEEL_RADIUS = 0.05 # From puzzlebot_gazebo/urdf/parameters.xacro

class DifferentialController:
    def __init__(self, wheel_dist=0.2, wheel_radius=0.3):
        """
        Arguments
        --------------
        wheel_dist  : float
           The distance between the two wheels
        wheel_radius  : float
        """
        self.cmd_vel_listener = rospy.Subscriber('/cmd_vel', Twist,
                                              self.cmd_vel_callback)
        self.wR_pub = rospy.Publisher('/right_wheel_velocity_controller/command' , Float64, queue_size=1 )
        self.wL_pub = rospy.Publisher('/left_wheel_velocity_controller/command' , Float64, queue_size=1 )
        self.rate = rospy.Rate(20.0)
        self.cmd = None
        self.wd = wheel_dist
        self.wr = wheel_radius
        
    def cmd_vel_callback(self, msg):
        """Called when a new message is available. """
        self.cmd = msg

    def run(self):
        while not rospy.is_shutdown():

            if self.cmd is not None:
                #--------------------------------------------------------------
                # Your code here
                # 1) Convert the information in the Twist cmd to angular velocities
                # of the wheels.
                # 2) Publish the desired angular velocities to each of the wheel PIDs

                pass
                #--------------------------------------------------------------
                #--------------------------------------------------------------
                
                
            self.rate.sleep()        
            

def canonical2diffdrive(w, v, r=WHEEL_RADIUS, d=WHEEL_DISTANCE):
    """
    Returns the angular velocities of the two wheels, respectively, of a differential
    drive robot corresponding to a certain angular and linear velocity of the canonical model.
    
    Arguments
    ---------
       w  :  float
             The angular velocity in rad/s.
       v  :  float
             The linear velocity in m/s.
             
    Returns
    -------
       wL  :  float
              The angular velocity of the left wheel in rad/s
       wR  :  float
              The angular velocity of the right wheel in rad/s
              
    Tests
    -----
    
    1) Only linear velocity gives same wheel velocities.
    >>> wL, wR = canonical2diffdrive(0, 1)
    >>> wL == wR
    True

    2) Only angular velocity gives same wheel velocities, but opposite sign.
    >>> wL, wR = canonical2diffdrive(1, 0)
    >>> wL == -wR
    True

    """
    
    #--------------------------------------------------------------
    # Your code here
    wL = 1.0
    wR = 2.0
    #--------------------------------------------------------------
    #--------------------------------------------------------------

    
    return (wL, wR)
        


if __name__ == '__main__':
    if len(sys.argv) > 1:
        if sys.argv[1] == "--test":
            import doctest
            doctest.testmod()
            sys.exit(0)

    rospy.init_node('Differentialcontroller')
    DifferentialController().run()

    


