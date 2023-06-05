# Puzzlebot Navigation with LiDAR and Aruco detection

Collaborators:
- Eva Denisse Vargas Sosa [@denvars](https://github.com/denvars)
- Daniela Avila Luna [@DannyAvilaL](https://github.com/DannyAvilaL)
- Roberto David Manzo González [@robertomanzo2203](https://github.com/robertomanzo2203)

This repository is detined for the puzzlebot robot with a camera and a LiDAR sensor mounted. The main objective of this project is navigate through a series of points defined and estimate position based on the dynamic model and Kalman Filter correction using defined Arucos at specific positions in the world. 

**Launch instructions to launch in separated terminals:**

To launch the challenge world. *This launch includes the odometry estimation with Kalman Filter, publish the frame of the real robot in gazebo and launch the first aruco detect launch*
```
$ roslaunch bloque navigation_world.launch 
```

To launch the aruco detection node. **This might trigger a warning/error in the previous terminal launch since it launches the node with the same name. It is normal.**
```
$ rosrun aruco_detect aruco_detect /camera/compressed:=/camera/image_raw/compressed /camera_info:=/camera/camera_info
```

To launch the navigation node.
```
$ rosrun bloque navigation.py
```


