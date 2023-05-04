#!/usr/bin/env python  
import sys
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class RightHandRuleController:

    def __init__(self, wall_dist=0.5, w_max = np.pi, v_max=0.4 ):
        """
        Arguments
        --------------
        wall_dist  : float
           Desired distance to wall
        w_max  : float
           Max angular velocity
        v_max  : float
           Max linear velocity
        """
        self.scan_listener = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_listener = rospy.Subscriber('/odometry', Odometry, self.odom_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel' , Twist, queue_size=1 )
        self.rate = rospy.Rate(10.0)
        self.wall_dist = wall_dist
        self.w_max = w_max
        self.v_max = v_max
        self.scan = None
        self.odom = None

    def scan_callback(self, msg):
        """Called when a new scan is available from the lidar. """
        self.scan = msg

    def odom_callback(self, msg):
        """Called when the odometry is updated"""
        self.odom = msg

    def go_to_wall(self):
        """ Go straight ahead until at desired distance from a wall. """
        msg = Twist()
        msg.linear.x = 0.2
        while not rospy.is_shutdown():
            if self.scan is not None:
                if self.scan.ranges[360] < self.wall_dist:
                    msg.linear.x = 0
            self.vel_pub.publish(msg)

        
    def follow_right_hand_wall(self):
        found_wall = False
        w = 0
        v = 0.5
       

        if self.scan is not None:

            distance_ahead = get_distance_in_sector(self.scan,
                                                    0 - 2*np.pi/180,
                                                    0 + 2*np.pi/180)
            distance_to_right = get_distance_in_sector(self.scan,
                                                        np.pi/2 - 4*np.pi/180,
                                                        np.pi/2)
            #--------------------------------------------------------------
            #--------------------------------------------------------------
            alpha = find_wall_direction(self.scan)
            anguloDerechaDeseado = 0
            distanciaDerechaDeseado = 0.8
            #distanciaFrontalMaxima = 1
            
            distanciaDerecha = distance_to_right * np.cos(alpha)

            errorAngulo = anguloDerechaDeseado - alpha
            errorDistancia = distanciaDerechaDeseado - distanciaDerecha 

            kp_alpha = 0.9
            kp_dist = 1.2

            print(errorDistancia, "PARED error")

            if distance_ahead < 1.5:
                print("FRENTE PARED ===")
                v = 0.25
                w =  0.8
            else:
                w = (kp_alpha * errorAngulo) + (kp_dist * errorDistancia)

            limVelocidad = 1
            #saturacion
            if w > limVelocidad:
                w = limVelocidad
            elif w < -limVelocidad:
                w = -limVelocidad
            
            #rospy.loginfo("{} {} {}".format(errorAngulo, errorDistancia, w))
                            
            msg = Twist()
            msg.angular.z = w
            msg.linear.x = v
            #print(msg)
            self.vel_pub.publish(msg)
            
        self.rate.sleep()        

    def turn_to_point(self, p):
        """Turns towards the point indicated"""
        vel = Twist()

        thetaIdeal = np.arctan2(
            p[1] - self.odom.pose.pose.position.y, 
            p[0] - self.odom.pose.pose.position.x
            )

        # error del angulo
        thetaE = round(thetaIdeal - self.odom.pose.pose.orientation.w, 4)
        rospy.loginfo("Error Angulo: {}".format(thetaE))
        k_th = 0.2

        vel.angular.z = thetaE

        # rospy.loginfo("ODOMETRY TURN {}".format(self.odom.pose.pose.orientation.w))
        self.vel_pub.publish(vel)
        self.rate.sleep()   
            


def find_wall_direction(scan):
    """Assuming wall is on the right, finds the direction of the wall w.r.t
    the robot frame (x ahead, y to the left). The direction is returned
    as an angle, which is 0 if the wall is parallel to the heading of the
    robot and negative if the robot is heading away from the wall.

    Tests
    -----
    >>> scan = generate_test_scan(straight_wall=True)
    >>> wall_dir = find_wall_direction(scan)
    >>> np.abs(wall_dir) < 1e-6
    True
    """

    hip = get_distance_in_sector(scan, np.radians(45-1), np.radians(45))
    ady = get_distance_in_sector(scan, np.radians(90-1), np.radians(90))


    alpha = np.arctan2(hip * np.cos(np.deg2rad(45))-ady, 
                       hip*np.sin(np.deg2rad(45)))
    return alpha

    
def get_distance_in_sector(scan, start_angle, end_angle) :
    """Returns the distance in m in the given sector by taking the average of the
    range scans.
    
    Arguments
    ---------
    scan : LaserScan
    start_angle : float
        Start angle of sector, in radians, where 0 is straight ahead.
    end_angle : float
        End angle of sector, in radians, where 0 is straight ahead.

    """
    # Obtener el indice del rango mas cercano al angulo de inicio
    start_index = range_index(scan, start_angle)
    # Obtener el indice del rango mas cercano al angulo de fin
    end_index = range_index(scan, end_angle)
    # Slicing de la seccion de rangos
    sector_ranges = np.array(scan.ranges)
    sector_ranges = sector_ranges[end_index:start_index]
    # Calcular la distancia media en el sector
    return np.mean(sector_ranges)
    

def range_index(scan, angle):
    """Returns the index into the scan ranges that correspond to the angle given (in rad).
    If the angle is out of range, then simply the first (0) or last index is returned, no
    exception is raised.

    Arguments
    ---------
    scan : LaserScan
    angle : float
       Angle w.r.t the x-axis of the robot, which is straight ahead

    """
    min_angle = scan.angle_min
    max_angle = scan.angle_max
    size = len(scan.ranges)
    if angle > max_angle:
        return size -1
    elif angle < min_angle:
        return 0
    
    grados = np.linspace(max_angle, min_angle, size)
    index = (np.abs(grados-angle)).argmin()
    #print("INDICE", index)
    return index




if __name__ == '__main__':

    rospy.init_node('bug_zero_node')
    rhw = RightHandRuleController()
    while not rospy.is_shutdown():
        #rhw.go_to_wall()
        #rhw.follow_right_hand_wall()
        if rhw.odom is not None:
            rhw.turn_to_point((3,2))



