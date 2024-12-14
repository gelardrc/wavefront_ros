# wavefront_ros
Thats a package that implements wavefront algorithm in a occupancy grid via ROS2 messages.
---------------------------------------------
# How to install
> git clone https://github.com/gelardrc/wavefront_ros.git

# Parameters

- ~BUFEFR_RADIUS (default=1)
	Inflate your map objects by the size of it.
- ~start (default=[34,25])
	Start position on map. 
- ~goal (default = [29,44])
	Goal position on map.
- ~animated(default=False)
	If you want to see the path been construct on Rviz while wavefront is calculated.

# How to run 

> rosrun wavefront_ros wavefront_a_star.py 

# Examples

> roslaunch wavefront_ros example.launch

If everything goes right, you should see this on Rviz:

![mapa](https://github.com/gelardrc/wavefront_ros/blob/main/img/path.gif)


# To do list

- Improve origin map on animated
- Improve algorithm (sometimes it stops before finishing cpp)

# Obs 

- This package is a slice of bigger project, also hosted on github, that creates a whole multi-agent architeture for industrial inspecions.

