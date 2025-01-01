# wavefront_ros

A ROS package implementing the wavefront algorithm on an occupancy grid via ROS messages.

---

## How to Install

```bash
git clone https://github.com/gelardrc/wavefront_ros.git
```

## Parameters

- **`method`** (default = `classic`):  
  Specifies the wavefront algorithm variation to use:
  - `a*`: Wavefront combined with A* algorithm.
  - `ba*`: Wavefront with bidirectional A*.
  - `classic`: Classic wavefront with backtracking.
  - `random_walk`: Random walk strategy.

- **`BUFFER_RADIUS`** (default = `1`):  
  Inflates map objects by this size.

- **`start`** (default = `[34, 25]`):  
  Starting position on the map.

- **`goal`** (default = `[29, 44]`):  
  Goal position on the map.

- **`animated`** (default = `False`):  
  Enables path visualization in RViz while wavefront is being calculated.

- **`map`** (default = `True`):  
  If `True`, uses the package's default map.  
  To use a custom map, set this to `False` and run the `map_server` package in another terminal.

## How to Run

```bash
rosrun wavefront_ros wavefront_a_star.py
```

## Examples

```bash
roslaunch wavefront_ros example.launch
```

If everything runs correctly, you should see this in RViz:

![Path Visualization](https://github.com/gelardrc/wavefront_ros/blob/ros1_noetic/img/path.gif)

## To-Do List

- Replace global parameters with local ones to facilitate migration to ROS 2.
- Add `.yaml` configuration files for quicker setup.
- Implement the `random_walk` algorithm (in progress).
- Debug `results.py` (only runs in Google Colab?).

## Notes

This package is part of a larger project, also available on GitHub, which creates a multi-agent architecture for industrial inspections.
