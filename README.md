[<img height="23" src="https://github.com/lh9171338/Outline/blob/master/icon.jpg"/>](https://github.com/lh9171338/Outline) Astar
===
  
# 1. Introduction
>>This repository is a ROS package of A star algorithm implemented by python.

# 2. Usage
## 2.1 Subscribed Topics  
initialpose ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))  
>>Receive the start point via this topic.

move_base_simple/goal ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))   
>>Receive the target point via this topic.

## 2.2 Subscribed Services
static_map ([nav_msgs/GetMap](http://docs.ros.org/en/api/nav_msgs/html/srv/GetMap.html)) 
>>Receive the map via this service.

## 2.3 Published Topics  
inflate_map ([nav_msgs/OccupancyGrid](http://docs.ros.org/kinetic/api/nav_msgs/html/msg/OccupancyGrid.html))  
>>Publish the inflation map via this topic.  

nav_path ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))  
>>Publish the navigation path via this topic.

## 2.4 Parameters  
~euclidean (bool; default: "true")  
>>Using Euclidean distance or Manhattan distance When calculating the H value.

~occupyThresh (int; default: -1)  
>>Threshold of the image binarization. When OccupyThresh is less than zero (OccupyThresh < 0), using Otsu method to generate threshold.

~inflateRadius (double; defalut: -1)  
>>InflateRadius is the inflation radius (unit: m). When InflateRadius is less than or equal to zero (InflateRadius <= 0), no inflation operation is taken.

~rate (int; default: 10)  
>>The rate of publishing inflate_map topic.

## 2.5 Example  
```
roslaunch astar astar.launch
```

# 3. Result  
![image](https://github.com/lh9171338/Astar/blob/python/figure/result.png)
