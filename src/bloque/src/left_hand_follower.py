#!/usr/bin/env python  
import sys
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

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
        self.scan_listener = rospy.Subscriber('/scan', LaserScan,
                                              self.scan_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel' , Twist, queue_size=1 )
        self.rate = rospy.Rate(10.0)
        self.wall_dist = wall_dist
        self.w_max = w_max
        self.v_max = v_max
        self.scan = None

    def scan_callback(self, msg):
        """Called when a new scan is available from the lidar. """
        self.scan = msg

    def go_to_wall(self):
        """ Go straight ahead until at desired distance from a wall. """
        #--------------------------------------------------------------
        # Your code here
        #--------------------------------------------------------------
        #--------------------------------------------------------------
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
        v = 0.05 # 0.6
        while not rospy.is_shutdown():
            #print(self.scan)
            if self.scan is not None:
                #--------------------------------------------------------------
                # Your code here
                # 1) Check distance to wall/obstacle in front and to the right.
                # 2) Based on the information in 1) calculate a suitable angular
                #    velocity, w.
                # 3) Publish the Twist message to steer the puzzlebot.

                distance_ahead = get_distance_in_sector(self.scan,
                                                        0 - 2*np.pi/180,
                                                        0 + 2*np.pi/180)
                distance_to_right = get_distance_in_sector(self.scan, 
                                                           #np.pi/2 - 4*np.pi/180,  #86
                                                           #np.pi/2                 #90
                                                           np.deg2rad(0 - 90),
                                                           np.deg2rad(0 - 86)
                                                           )
                #--------------------------------------------------------------
                #--------------------------------------------------------------
                alpha = find_wall_direction(self.scan)
                anguloDerechaDeseado = 0
                distanciaDerechaDeseado = -0.3
                #distanciaFrontalMaxima = 1
                
                #print("DISTANCIA A LA IZQUIERDA: ", distance_to_right)
                distanciaDerecha = distance_to_right * np.cos(alpha)

                errorAngulo = anguloDerechaDeseado + alpha
                errorDistancia = distanciaDerechaDeseado + distanciaDerecha

                print("Angulo", alpha, "Error angulo", errorAngulo)
                print("DistanciaActual", distanciaDerecha, "Error distancia", errorDistancia)

                kp_alpha = 3
                kp_dist = 2
                #print( self.scan.ranges[360])
                if get_distance_in_sector(self.scan, -3, 3) < 1.5:
                    print("FRENTE PARED ===")
                    v = 0.3
                    w = -0.5
                else:
                    #pass
                    v = 0.4
                    w = (kp_alpha * errorAngulo) + (kp_dist * errorDistancia)
                #w = (kp_alpha * errorAngulo) + (kp_dist * errorDistancia)

                limVelocidad = 1.5
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

    #hip = get_distance_in_sector(scan, np.radians(45-1), np.radians(45))
    #ady = get_distance_in_sector(scan, np.radians(90-1), np.radians(90))

    hip = get_distance_in_sector(scan, np.radians(-46), np.radians(-44))
    ady = get_distance_in_sector(scan, np.radians(-90), np.radians(-86))


    alpha = np.arctan2(hip * np.cos(np.deg2rad(45))-ady, 
                       hip*np.sin(np.deg2rad(90)))
    #print(hip, ady, alpha, "FIND WALL")
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
    #--------------------------------------------------------------
    # Your code here. For documentation of the LaserScan message:
    # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
    #
    # 1) Find the indices into scan.ranges corresponding to the start_ange and end_angle
    # 2) Compute the average range in the sector using np.mean()
    #--------------------------------------------------------------
    # Obtener el indice del rango mas cercano al angulo de inicio
    #print(start_angle, end_angle, scan.angle_min, scan.angle_max, scan.ranges)
    start_index = range_index(scan, start_angle)
    # Obtener el indice del rango mas cercano al angulo de fin
    end_index = range_index(scan, end_angle)
    # Slicing de la seccion de rangos
    #print(start_index, end_index, "START AND END INDEXES")
    #print(scan.ranges)
    sector_ranges = np.array(scan.ranges)
    sector_ranges = sector_ranges[end_index:start_index]
    # Calcular la distancia media en el sector
    #print(sector_ranges, np.mean(sector_ranges), "====")
    #print(sector_ranges, "Sector ranges")
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

    #[90....0.......-90] -> funcion lineal y=x-90
    #[0,1,2,3,4.....720]


def generate_test_scan(straight_wall=False):
    """Function used for testing. Will create and return a LaserScan message"""
    scan = LaserScan()
    scan.angle_min = -np.pi/2
    scan.angle_max = np.pi/2
    num_scans = 720 # 4 lines per degree
    scan.angle_increment = np.pi/num_scans
    scan.range_min = 0.1
    scan.range_max = 30

    scan.ranges = np.arange(0, 720.0) / 10.0
    if straight_wall: # The wall is on the right at a distance of 10m
        scan.ranges[:400] = scan.range_max
        dth = np.arange(0, 320)*scan.angle_increment
        for i in range(320):
            scan.ranges[719-i] = 10/np.cos(dth[i])

    return scan

if __name__ == '__main__':
    if len(sys.argv) > 1:
        if sys.argv[1] == "--test":
            import doctest
            doctest.testmod()
            sys.exit(0)

    rospy.init_node('Follow_right_hand_wall')
    rhw = RightHandRuleController()
    #rhw.go_to_wall()
    rhw.follow_right_hand_wall()
    


