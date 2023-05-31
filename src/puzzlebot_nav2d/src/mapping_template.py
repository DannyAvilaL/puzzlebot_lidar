#!/usr/bin/env python  
""" Node that generates a map of the environment based on the laser scan data.
and the odometry data.
Author: kjartan@tec.mx (Kjartan Halvorsen) with help from github copilot
Notes.
1) The scan data give information about free space as well as obstacles. Each ray in the scan will cover a number
of pixels in the map. The map should be updated by setting the pixels covered by the ray to 0 (free) and the last pixel
to occupied (100). The map should be updated only if the ray range is less than the max_range of the scan.
2) You should determine the number of points in each scan ray by multiplying the range of the ray by the map resolution.
Then you convert these points (each corresponding to a pixel) from a robot frame to a map frame using the odometry data.
3) The map should be updated only if the robot has moved a certain distance since the last update. This is to
avoid updating the map too often, since it is a somewhat expensive operation.
4) It can be more efficient to use numpy arrays for the rigid transformations needed to convert scans
to map coordinates. To make this work, you need to convert the geometry_msgs/TransformStamped to a numpy array.
See https://answers.ros.org/question/332407/transformstamped-to-transformation-matrix-python/
With a transform matrix T, you can transform a number of points at once by collecting the points in a numpy array
and multiplying the array with T.
To use numpify, you need to install the package ros-meldic-ros-numpy.
"""
import sys
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from ros_numpy import numpify

class Mapper:
    def __init__(self, map_width, map_height, map_resolution):
        """
        Arguments
        ---------
        map_width : float
            Width of map in pixels (x-axis)
        map_height : float
            Height of map in pixels (y-axis)
        map_resolution : float
            Resolution of map in meter per pixel
        """
        self.scan_listener = rospy.Subscriber('/scan', LaserScan,
                                              self.scan_callback)
        self.odom_listener = rospy.Subscriber('/odom', Odometry,
                                              self.odom_callback)
        self.map_pub = rospy.Publisher('/map' , OccupancyGrid, queue_size=1 )
        self.rate = rospy.Rate(5.0)
        self.map = OccupancyGrid()
        self.map.info.map_load_time = rospy.Time.now()
        self.map.info.resolution = map_resolution
        self.map.info.width = map_width
        self.map.info.height = map_height
        self.map.info.origin.position.x = -(map_width*map_resolution)/2.0
        self.map.info.origin.position.y = (map_height*map_resolution)/2.0
        self.map.info.origin.position.z = 0.0
        self.map.info.origin.orientation.x = 1.0
        self.map.info.origin.orientation.y = 0.0
        self.map.info.origin.orientation.z = 0.0
        self.map.info.origin.orientation.w = 0.0

        self.map.data = np.zeros(map_width*map_height, dtype=np.int8)
        self.map.data[:] = -1 # Unknown
        self.map2d = np.zeros((map_width, map_height), dtype=np.int8) # For computation
        self.scan = None
        self.odom = None


    def scan_callback(self, msg):
        """Called when a new scan is available from the lidar. """
        self.scan = msg
        if self.scan is not None:
            self.scan.ranges = np.array(map(lambda x: x if x != np.inf else self.scan.range_max, self.scan.ranges)) 

    def odom_callback(self, msg):
        """Called when a new odometry message is available. """
        self.odom = msg

    def mapit(self):
        while not rospy.is_shutdown():
            #print(self.scan)
            if self.scan is not None and self.odom is not None:
                #--------------------------------------------------------------
                # Your code here
                # 1) For each ray in the scan, calculate the corresponding
                #    position in the map by transforming the ray from the robot
                #    frame to the map frame, using the odometry data. 
		        #    It is convenient to define the map frame as having its origin
		        #    in the pixel (0,0), and directions corresponding to the 
		        #    rows and pixels of the map (occupancy grid).
                #    is defined as the frame of the first laser scan, when the robot
                #    is initialized.
                # 2) If the ray range is less than max_range, then set the map pixel
                #    corresponding to the end point of the ray to 100 (occupied).
            
                # 3) Set pixels along the ray to 0 (free).
                #--------------------------------------------------------------
                #--------------------------------------------------------------
                print("CORRIENDO MAPA")
                orig_m, xy_m = scan_to_map_coordinates(self.scan, self.odom, self.map.info.origin)

                for xy_ in xy_m:
                    #print(xy_)
                    ray_to_pixels(orig_m[0], orig_m[1], xy_[0], xy_[1], self.map.info.resolution, self.map2d)

                # Publish the map
                np.copyto(self.map.data,  self.map2d.reshape(-1)) # Copy from map2d to 1d data, fastest way
                self.map.header.stamp = rospy.Time.now()
                self.map_pub.publish(self.map)
            self.rate.sleep()        
            

def ray_to_pixels(xr, yr, x, y, map_resolution, map):
    """ Set the pixels along the ray with origin (xr,yr) and with range ending at (x,y) to 0 (free) 
    and the end point to 100 (occupied).

    You should determine the number of points in each scan ray by multiplying 
    the range of the ray by the map resolution.
    Then you convert these points (each corresponding to a pixel)

    Arguments
    ---------
    xr : float
        x-coordinate of the robot in the map frame
    yr : float
        y-coordinate of the robot in the map frame
    x : ndarray
        x-coordinates of the scan in the map frame
    y : ndarray
        y-coordinates of the scan in the map frame
    map_resolution : float
        Resolution of map in meter/pixel
    map : ndarray
        The map as a 2d numpy array
    Tests
    ------
    >>> mapmsg = test_map()
    >>> map = np.zeros((mapmsg.info.width, mapmsg.info.height), dtype=np.int8)
    >>> map[:] = -1
    >>> xr = 6.0
    >>> yr = 3.0
    >>> # Test 1 - ray from (6,3) to (7,3)
    >>> x = 7.0
    >>> y = 3.0
    >>> ray_to_pixels(xr, yr, x, y, mapmsg.info.resolution, map)
    >>> map[6, 12] == 0
    True
    >>> map[6, 13] == 0
    True
    >>> map[6, 14] == 100
    True
    >>> # Test 2 - ray from (6,3) to (6,2)
    >>> x = 6.0
    >>> y = 2.0
    >>> ray_to_pixels(xr, yr, x, y, mapmsg.info.resolution, map)
    >>> map[6, 12] == 0
    True
    >>> map[5, 12] == 0
    True
    >>> #map[4, 12] == 100*[(0.0, -1.0), (1.0, 0.0), (0.0, 1.0)]
    True
    >>> # Test 3 - ray from (6,3) to (5,3)
    >>> x = 5.0
    >>> y = 3.0
    >>> ray_to_pixels(xr, yr, x, y, mapmsg.info.resolution, map)
    >>> map[6, 12] == 0
    True
    >>> map[6, 11] == 0
    True
    >>> map[6, 10] == 100
    True
    >>> #map
    """

    v = np.array([x-xr, y-yr]) #vector / linea entre la posicion del robot a la del punto
    n_pixels = np.linalg.norm(v)/map_resolution #pixeles por vector
    v = v/n_pixels # de los metros calculados, a cuant x0 = int(round(x0/resolution))
    xr, yr = int(round(xr/map_resolution)), int(round(yr/map_resolution))
    x, y = int(round(x/map_resolution)), int(round(y/map_resolution))
 
    dx, dy = x - xr, y - yr

    x_ = 1 if dx > 0 else -1
    y_ = 1 if dy > 0 else -1

    dy = abs(dy)
    dx = abs(dx)
    

    if dx > dy:
        xx, xy, yx, yy = x_, 0, 0, y_
    else:
        dx, dy = dy, dx
        xx, xy, yx, yy = 0, y_, x_, 0

    d = 2 * dy - dx
    y_inc = 0

    for i in range(dx + 1):
        x_coord = yr + i * xy + y_inc * yy
        y_coord = xr + i * xx + y_inc * yx
        map[x_coord, y_coord] = 0
        
        if d >= 0:
            y_inc += 1
            d -= 2 * dx
        d += 2 * dy

    map[y, x] = 100
    #print("MAP", map[2:10, 9:15],)
    #print(x_, y_, len(map))

def scan_to_map_coordinates(scan, odom, origin):
    """ Convert a scan from the robot frame to the map frame.
    Arguments
    ---------
    scan : LaserScan
        The scan to convert
    odom : Odometry
        The odometry message providing the robot pose
    origin : Pose
        The pose of the map in the odom frame
    Returns
    -------
    (xr, yr) : tuple of floats
        The position of the robot in the map frame
    xy : list-like
        list of tuples (x,y) with the coordinates of the scan end points in the map frame
    Tests
    -----
    >>> # Test 1 - With map origin at (0,0), no rotation of the map, and robot at (1,2) in odom frame
    >>> scan = test_laser_scan()
    >>> odom = test_odometry()
    >>> origin = Pose()
    >>> orig, xy = scan_to_map_coordinates(scan, odom, origin)
    >>> np.allclose(orig, (1.0, 2.0))
    True
    >>> np.allclose(xy,[(2.0, 2.0), (1.0, 3.0), (0.0, 2.0)])
    True
    >>> # Test 2 - With map origin at (-5, 5), map frame rotated pi about x, and robot at (1,2) in odom frame
    >>> map = test_map()
    >>> origin = map.info.origin
    >>> orig, xy = scan_to_map_coordinates(scan, odom, origin)
    >>> np.allclose(orig, (6.0, 3.0))
    True
    >>> np.allclose(xy,[(7.0, 3.0), (6.0, 2.0), (5.0, 3.0)])
    True
    >>>
    """

    T_ob = numpify(odom.pose.pose)  # Transform from odom to base_link 4x4
    T_om = numpify(origin) 
    T_mo = np.linalg.inv(T_om) #4x4
    T_mb = np.dot(T_mo, T_ob)
    TT= np.eye(3) #3x3
    TT[:2, :2] = T_mb[:2, :2]
    TT[:2, 2] = T_mb[:2, 3]

    #------------------------------------------------------
    # Your code here
    # Transform all the scan end points to the map frame
    #scan_endpoints = scan.ranges
    angulo = scan.angle_min
    posiciones = []
    
    for rayo in scan.ranges:
        x, y = polar_to_cartesian(rayo, angulo)
        pos = np.array([x, y, 1])
        posiciones.append(np.dot(TT, pos)[:2])
        angulo += scan.angle_increment
    
    return TT[:2, 2], posiciones

def polar_to_cartesian(r, th):
    """ Convert a polar coordinate to a cartesian coordinate.
    Arguments
    ---------
    r : float
        The radius
    th : float
        The angle
    Returns
    -------
    (x, y) : tuple of floats
        The cartesian coordinates
    Tests
    -----
    >>> x, y = polar_to_cartesian(1.0, 0.0)
    >>> np.allclose(x, 1.0)
    True
    >>> np.allclose(y, 0.0)
    True
    >>> x, y = polar_to_cartesian(1.0, np.pi/2)
    >>> np.allclose(x, 0.0)
    True
    >>> np.allclose(y, 1.0)
    True
    >>> x, y = polar_to_cartesian(1.0, np.pi)
    >>> np.allclose(x, -1.0)
    True
    >>> np.allclose(y, 0.0)
    True
    """

    #------------------------------------------------------
    # Your code here
    # Convert the polar coordinate to a cartesian coordinate
   
    # if th >= np.deg2rad(90):
    #     x = 0
    #     y = r
    # elif th <= -np.deg2rad(90):
    #     x = 0
    #     y = -r
    # else:
    x = r * np.cos(th) 
    y = r * np.sin(th) 
    #-----------------------------------------------------
    return x, y

def test_laser_scan():
    """ Create a simple test LaserScan message.
    There are 3 rays, to the left, straight ahead and to the right.
    Each ray has range 1.0. """
    scan = LaserScan()
    scan.header.frame_id = 'base_link'
    scan.angle_min = -np.pi/2
    scan.angle_max = np.pi/2
    scan.angle_increment = np.pi/2
    scan.range_min = 0.0
    scan.range_max = 10.0
    scan.ranges = [1.0, 1.0, 1.0]
    return scan

def test_odometry():
    """ Create a test Odometry message. """
    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.pose.pose.position.x = 1.0
    odom.pose.pose.position.y = 2.0
    odom.pose.pose.position.z = 0.0
    # Quaternion for 90 degree rotation around z-axis. So the robot is facing in the y-direction.
    odom.pose.pose.orientation.x = 0.0
    odom.pose.pose.orientation.y = 0.0
    odom.pose.pose.orientation.z = np.sin(np.pi/4)
    odom.pose.pose.orientation.w = np.cos(np.pi/4)
    return odom

def test_map():
    """ Create a test map. """
    map = OccupancyGrid()
    map.header.frame_id = 'map'
    map.info.resolution = 0.5
    map.info.width = 20
    map.info.height = 20
    # Position of the map origin in the odom frame.
    map.info.origin.position.x = -5.0
    map.info.origin.position.y = 5.0
    # Rotation of pi around x-axis. So the map is upside down.
    map.info.origin.orientation.w = 0.0
    map.info.origin.orientation.x = 1.0
    return map

if __name__ == '__main__':
    if len(sys.argv) > 1:
        if sys.argv[1] == "--test":
            import doctest
            doctest.testmod()
            #scan_to_map_coordinates()
            sys.exit(0)


    rospy.init_node('Mapper')
    width = rospy.get_param("/mapper/width", 400)
    height = rospy.get_param("/mapper/height", 400)
    resolution = rospy.get_param("/mapper/resolution", 0.1) # meters per pixel

    Mapper(width, height, resolution).mapit()




