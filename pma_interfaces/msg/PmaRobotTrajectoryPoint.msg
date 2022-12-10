# A point in a joint-space trajectory with a corresponding pressure required
# for a PMA robot, describes the point for each joint.

# The joint positions, velocities, accelerations, and efforts (torques, for
# this type of robot)
trajectory_msgs/JointTrajectoryPoint joint_space

# The pressures for both the bicep and tricep groups
float64[] pressures

# The time when this point was commanded
builtin_interfaces/Time time_commanded
