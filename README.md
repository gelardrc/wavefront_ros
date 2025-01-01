# wavefront_ros

Thats a package that implements wavefront algorithm in a occupancy grid via ROS messages.
-----------------------------------------------------------------------------------------

# How to install

> git clone https://github.com/gelardrc/wavefront_ros.git

# Parameters

- method (default = classic)
  a* - Wavefront + a star
  ba* - Wavefront + bidirectional a star
  classic - Wavefront classical backtracking
  random_walk -
- BUFEFR_RADIUS (default=1)
  Inflate your map objects by the size of it.
- start (default=[34,25])
  Start position on map.
- goal (default = [29,44])
  Goal position on map.
- animated(default=False)
  If you want to see the path been construct on Rviz while wavefront is calculated.
- map (default=True)
  If you want to run your own map, set to false and run in another terminal map_server package

# How to run

> rosrun wavefront_ros wavefront_a_star.py

# Examples

> roslaunch wavefront_ros example.launch

If everything goes right, you should see this on Rviz:

![mapa](https://github.com/gelardrc/wavefront_ros/blob/main/img/path.gif)

# To do list

- Change all global parameters to local, that will help when upgrade to ros2.
- Construct some .yaml files to set configs faster.
- Implement randomwalk algorithm (working)
- debug results.py ( only in collab ? )

# Obs

- This package is a slice of bigger project, also hosted on github, that creates a whole multi-agent architeture for industrial inspecions.
