A convenience wrapper that handles the include and link for opencv automatically for catkin packages.

Make sure that the ROS catkin package "opencv2" is somewhere in your workspace, then add 
```
<build_depend>opencv2_catkin</build_depend>
```
to your package. OpenCV will be automatically found and linked.
