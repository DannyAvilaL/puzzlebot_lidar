#!/usr/bin/env python  
import sys
import rospy
import numpy as np
from functools import partial
import tf2_ros
import geometry_msgs.msg
from tf2_geometry_msgs import PointStamped
import py_trees

def go_to_point_controller(x, y, vmax, Kth, alpha):
    """
    Calculates the desired linear- and angular velocities to move the robot to a point.
    Used for the final step in the pure pursuit to approach and stay at the goal position.

    Arguments
    ---------
    x     :  float
      The x-coordinate of the goal in the reference frame of the robot, in m
    y     :  float
      The x-coordinate of the goal in the reference frame of the robot, in m
    vmax  :  float
      The maximum linear velocity, in m/s
    Kth   :  float
      Gain for the direction controller
    alpha :  float
      Parameter for calculating the linear velocity as it approaches the goal

    Returns
    ---------
    w   :  float
      The angular velocity in rad/s
    v   :  float
      The linear velocity in m/s
    d   :  float
      The distance to the goal

    """

    #---------------------------------------------------------
    # YOUR CODE HERE
    w = 0.0
    d = 0.0
    v = 0.0
    #---------------------------------------------------------
    
    return w, v, d

def steer_towards_point_controller(x, y, v):
    """
    Given an intermediate goal point and a (constant) linear velocity, calculates the
    angular velocity that will steer the robot towards the goal such that the robot
    moves in a circular arc that passes through the intermediate goal point.

    Arguments
    ---------
    x     :  float
      The x-coordinate of the goal in the reference frame of the robot, in m
    y     :  float
      The x-coordinate of the goal in the reference frame of the robot, in m
    v     :  float
      The linear velocity in m/s

    Returns
    ---------
    w   :  float
      The angular velocity in rad/s
    """

    #---------------------------------------------------------
    # YOUR CODE HERE
    w = 0.0
    #---------------------------------------------------------

    return w

def get_goal_point(p0, p1, L):
    """
    Returns the intermediate goal point for the pure pursuit algorithm. If no point
    on the line going through p0 and p1 is at distance L from the origin, then the
    returned beta should be a nan.

    Arguments
    ---------
    p0  :  array-like (2,)
         The current waypoint.
    p1  :  array-like (2,)
         The next waypoint.
    L   :  float\n",
         The look-ahead distance

    Returns\n",
    -------\n",
    pg   :  ndarray (2,)
          The intermediate goal point
    beta :  float
           The value giving the position of the goal point on the line connectin p0 and p1.
    """

    p0 = np.asarray(p0)
    p1 = np.asarray(p1)
    w = p1-p0

    a = np.dot(w, w)
    b = 2*np.dot(p0, w)
    c = np.dot(p0, p0) - L**2

    d = b**2 - 4*a*c
    if d < 0:
        pg = p0
        beta = np.nan
    else:
        #---------------------------------------------------------
        # YOUR CODE HERE
        beta = 0.0
        pg = [0,0]
        #---------------------------------------------------------

    return pg, beta
    
class Go2Point(py_trees.behaviour.Behaviour):
    """
    Takes a point to go to as input and will go towards that point until within
    given distance.
    """
    def __init__(self, name="Go2Point"):
        """
        Default construction.
        """
        super(Go2Point, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self, goal, distance, cmd_vel_pub, tf_buffer, robot_frame='base_link', 
              vel_max=0.6, K_theta=4.0, alpha=4.0):
        """
        Arguments
        ---------
          goal        :  PointStamped
             The goal point.
          distance    :  float
             When inside this distance, Success is returned.
          cmd_vel_pub :  Publisher 
             Publisher that publishes to the /cmd_vel topic
          tf_buffer   :  tf2_ros.Buffer
             Transform buffer for transforming the waypoints.
          robot_frame :  String
             The name of reference frame of the robot.
          vel_max     :  float
             The maximum linear velocity to command.
          K_theta     :  float
             The gain for the angular part of the controller.
          alpha       :  float
             The parameter for setting the linear velocity.
        
        """
        self.goal = goal
        self.dist = distance
        self.cmd_vel_pub = cmd_vel_pub
        self.tf_buffer = tf_buffer
        self.robot_frame = robot_frame
        self.vel_max = vel_max
        self.K_theta = K_theta
        self.alpha = alpha

        self.msg = geometry_msgs.msg.Twist()
        
        return True
        
    def initialise(self):
        """
        Since simple proportional control, no initialization needed. In case
        of PI, PD, or PID, the controller would have some state that should
        be initialized here.
        """
        pass
    
    def update(self):
        """
        Do the work:
        1) Transform waypoint to robot-centric frame
        2) Compute the control signal
        3) Publish the control signal
        """

        #----------------------------------------------------
        # YOUR CODE HERE
        # Transform the point to the robot reference frame
        self.goal.header.stamp = rospy.Time(0)
        goal_b = self.tf_buffer.transform(...)
        #----------------------------------------------------

        w, v, dist = go_to_point_controller(goal_b.point.x, goal_b.point.y,
                                            self.vel_max, self.K_theta, self.alpha)

        if dist < self.dist:
            # Within the desired distance, so return success
            return py_trees.Status.SUCCESS
        else:
            #----------------------------------------------------
            # YOUR CODE HERE
            # Publish the velocity command to cmd_vel
            self.msg.linear.x = v # etc.
            #----------------------------------------------------
            
            return py_trees.Status.RUNNING
        
    def terminate(self, new_status):
        """
        Nothing to clean up
        """
        pass

class PursuitGoal(py_trees.behaviour.Behaviour):
    """
    Takes two waypoints (current and next) and goes toward a point on
    the line joining the two waypoints and at the look-ahead distance.
    Returns SUCCESS when the next intermediate goal point is past the
    next waypoint.
    Returns RUNNING when still moving along the trajectory between the
    two waypoints.
    Returns FAILURE if no point on the trajectory is at the
    look-ahead distance from the robot.
    """

    def __init__(self, name="PursuitGoal"):
        """
        Default construction.
        """
        super(PursuitGoal, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self, wp0, wp1, look_ahead_distance,
              cmd_vel_pub, tf_buffer, robot_frame='base_link', 
              vel_max=0.6):
        """
        Arguments
        ---------
          wp0        :  array-like (2,)
             The current waypoint.
          wp1        :  array-like (2,)
             The next waypoint.
          look_ahead_distance    :  float
             The main parameter of the pure pursuit algorithm
          cmd_vel_pub :  Publisher 
             Publisher that publishes to the /cmd_vel topic
          tf_buffer   :  tf2_ros.Buffer
             Transform buffer for transforming the waypoints.
          robot_frame :  String
             The name of reference frame of the robot.
          vel_max     :  float
             The maximum linear velocity to command.
        """
        self.wp0 = wp0
        self.wp1 = wp1
        self.L = look_ahead_distance
        self.cmd_vel_pub = cmd_vel_pub
        self.tf_buffer = tf_buffer
        self.robot_frame = robot_frame
        self.vel_max = vel_max

        self.msg = geometry_msgs.msg.Twist()
        
        return True
        
    def initialise(self):
        """
        Since simple proportional control, no initialization needed. In case
        of PI, PD, or PID, the controller would have some state that should
        be initialized here.
        """
        pass
    
    def update(self):
        """
        Do the work:
        1) Transform waypoints to robot-centric frame
        2) Compute the control signal
        3) Publish the control signal
        """
        
        #------------------------------------------------------------
        # YOUR CODE HERE
        # Transform the two waypoints to the robot reference frame
        self.wp0.header.stamp = rospy.Time(0)
        self.wp1.header.stamp = rospy.Time(0)
        wp0_b = self.tf_buffer.transform(...)
        wp1_b = self.tf_buffer.transform(...)
        #------------------------------------------------------------

        pg, beta = get_goal_point([wp0_b.point.x, wp0_b.point.y],
                                  [wp1_b.point.x, wp1_b.point.y],
                                  self.L)

        if beta > 1.0:
            # Succeeded in moving along the trajectory from current to next waypoint
            # Time to move on to next pair of waypoints
            return py_trees.Status.SUCCESS

        elif np.isnan(beta):
            # Not able to find point on trajectory at look-ahead-distance
            # from the robot. Means failure.
            return py_trees.Status.FAILURE

        else:
            w = steer_towards_point_controller(pg[0], pg[1], self.vel_max)
            #----------------------------------------------------
            # YOUR CODE HERE
            # Publish the velocity command to cmd_vel

            #----------------------------------------------------

            return py_trees.Status.RUNNING
        
    def terminate(self, new_status):
        """
        Nothing to clean up
        """
        pass

def create_behavior_tree(waypoints, frame, look_ahead_dist, vel_max,
                K_theta, alpha):
    """
    Constructs and returns the behavior tree.
    The tree has only one level with a sequence of nodes, all of which
    must succeed in order to perform the task.
    """
    # Setup stuff for ROS communication
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)
    cmd_vel_pub = rospy.Publisher('/cmd_vel' , geometry_msgs.msg.Twist, queue_size=1 )

    root = py_trees.composites.Sequence("PurePursuit")

    # Setup and add node handling the first waypoint. We should go towards this point
    # until it is at a distance within the look-ahead-distance from the
    # robot.
    g2p = Go2Point()
    g2p.setup(waypoints[0], look_ahead_dist, cmd_vel_pub, tf_buffer,
              vel_max=vel_max, K_theta=K_theta, alpha=alpha)
              
    root.add_child(g2p)

    # Add the nodes to handle pairs of waypoints
    for cwp_, nwp_ in zip(waypoints[:-1], waypoints[1:]):
        pg = PursuitGoal()
        pg.setup(cwp_, nwp_, look_ahead_dist, cmd_vel_pub,
                 tf_buffer, vel_max=vel_max)
        root.add_child(pg)


    #----------------------------------------------------------------------------
    # YOUR CODE HERE
    # Add the final node to go to the last waypoint, which is the final goal point
    #
    #----------------------------------------------------------------------------

    return root


if __name__ == '__main__':
    rospy.init_node('Pure_pursuit')
    L = rospy.get_param("/pure_pursuit/look_ahead_distance", 0.3)
    vmax = rospy.get_param("/pure_pursuit/vel_lin_max", 0.6)
    Kth = rospy.get_param("/pure_pursuit/K_theta", 4)
    alpha = rospy.get_param("/pure_pursuit/alpha", 4)
    frame = rospy.get_param("/pure_pursuit/frame")
    rate = rospy.get_param("/pure_pursuit/rate", 10)
    waypoints = rospy.get_param("/waypoints", [-4,-4, -4, 2])
    waypoints = np.reshape(waypoints, (-1, 2))
    waypoints_ps = []
    for wp_ in waypoints:
        p = PointStamped()
        p.header.frame_id = frame
        p.point.x = wp_[0]
        p.point.y = wp_[1]
        waypoints_ps.append(p)
    
    bt_tree = create_behavior_tree(waypoints_ps, frame, L, vmax,
                Kth, alpha)

    rosrate = rospy.Rate(rate)

    while not rospy.is_shutdown():
        bt_tree.tick_once()
        rosrate.sleep()
        


