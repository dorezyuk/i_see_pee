# Overview

`i_see_pee` is a 2d localization package. At its core it uses an icp algorithm
to determine the current robot position. Its main advantage is that it performs
one optimization step linear in time: *O(N)*.

# Whats the difference

There are many stellar packages for localization which rely on icp. 

- [libpointmatcher](https://github.com/ethz-asl/libpointmatcher)
- [cartographer](https://github.com/googlecartographer/cartographer)
- [dynamic_robot_localization](https://github.com/carlosmccosta/dynamic_robot_localization)

These are generic localization/slam packages. They support 2d and 3d 
environment, offer rich features for sensor processing and what not.
Which is nice. `i_see_pee` is different. It just offers 2d localization. But 
its smaller. And its **way faster**.

## The speed

A typical icp algorithm consists of two steps: 
- k nearest neighbors matching
- solving the least squares problem. 

The least squares problem is typically *O(N)*, which is nice. 
The k nearest neighbors is a little more expensive. Typically the lookup is 
implemented using a knn-tree. The lookup there comes at *O(D log M)*, where *D* 
is the dimension and *M* the size of the occupied points on the map. 

When aiming for a generic setup, there is little one can do. `i_see_pee` 
leverages the fact that all grid_maps are discrete and that the registration 
cloud (map) is known in advance. It pre-computes for cells close to walls the 
k nearest neighbors and stores them in a hash-table. When a sensor reading falls 
in one of these cells, all its k nearest neighbors can be retrieved in linear 
time.
 
# Compiling

You will require [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
for the compilation. Besides that the package can be compiled as a standard
ros-package:

```
cd catkin_ws/src
git clone https://github.com/dorezyuk/i_see_pee.git
cd catkin_ws/src
catkin build --this
```

Depending on your processor you should enable 
[vectorization](http://eigen.tuxfamily.org/index.php?title=FAQ#How_can_I_enable_vectorization.3F) for Eigen.

# Configuring

## Subscriptions

#### `map/topic`
- topic under which the static map is published
- type: `string`
- default: `map`
- message_type: [nav_msgs::OccupancyGrid](http://docs.ros.org/diamondback/api/nav_msgs/html/msg/OccupancyGrid.html)

#### `scan/topic`
- topic under which the sensor data is published
- type: `string`
- default: `scan`
- message_type: [sensor_msgs::LaserScan](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html)

## Frames

#### `odom/map_frame`
- frame_id which defines the global frame.
- type: `string`
- default: `map`

#### `odom/base_frame`
- frame_id which defines the base frame. You can use either only odometry, 
which is provided by the differential_drive controller or fuse imu and 
odometry to improve the initial guess. 
- type: `string`
- default: `base_link`

#### `odom/base_frame`
- frame_id of the odom frame of the robot.
- type: `string`
- default: `odom`

## Mapping parameters

#### `map/k`
- number of neighbors to consider
- type: `int`
- default: 5 (min: 1, max: 10) 

#### `map/radius`
- number of cells to consider from an occupied cell. This massively determines
the size of the hash map
- type: `int`
- default: 5 (min: 0, max: 10) 

## Optimization parameters

#### `icp/t_norm`
- translation norm in meter defining convergence of the icp: 
if the generated transform is smaller than `icp/t_norm` and `icp/r_norm` the 
icp is said to be converged.
- type: `float`
- default: 0.01 (min: 0.01)

#### `icp/r_norm`
- rotation norm defining convergence (see [`icp/t_norm`])
- type: `float`
- default: 0.01 (min: 0.01)

#### `icp/max_iter`
- defines the maximum number of iterations of the icp optimization. The looping
stops, if the norm of the generated correction transform is smaller then 
`icp/t_norm` and `icp/r_norm`
- type: `int`
- default: 100 (min: 1, max: 100)

#### `icp/sample_percentage`
- in order to speed up the computation one can reduce the point cloud size by
applying random sampling. The parameter defines how much percent of the original
point cloud should be used for scan matching
- type: `float`
- default: 0.3 (min: 0, max: 1)
 
# Understanding

When you want to play with the code, you should be lucky: `i_see_pee` is very 
small (less then 1000 loc currently). As you might find, everything related
to specific ros-types (msgs, tf) is shielded away behind an interface.

The main files are undoubtedly `i_see_pee.hpp`/`cpp`. They contain several 
namespaces: 

- `map`: everything related to the static map. This includes a ros-interface 
and the algorithm to extract knns from a grid and to store them in a hash-map
- `scan`: everything related to the sensor data. This includes the
ros-interface to `sensor_msgs::LaserScan` and some caching to convert it to 
some internal representation. 
- `odom`: everything related to frames/odometry. 
- `icp`: everything related to the math part (which was heavily borrowed from 
[libpointmatcher](https://github.com/ethz-asl/libpointmatcher))

# Performance

As what I can tell from my machine, the i_see_pee package requires roughly 10% 
of CPU-resources of the [AMCL](https://github.com/ros-planning/navigation/tree/melodic-devel/amcl). 
But this does not tell very much, of course. 

**Let me know how your experience was with the package.** 

