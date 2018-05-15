# DWAPlanner
This integrates the DWA with the ROS.

It subscribes the belief from the odometry/filtered topic and publishes the velovity to be followed to the /cmd_vel topic

It also subscribes to the /map topic to get the cost map of the arena.
