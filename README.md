# puzzlebot

This repository is for the puzzlebot robot.

**Orden para lanzar los nodos en terminales separadas:**

```
$ roslaunch bloque navigation_world.launch 
```

```
$ rosrun aruco_detect aruco_detect /camera/compressed:=/camera/image_raw/compressed /camera_info:=/camera/camera_info
```
```
$ rosrun bloque navigation.py
```


