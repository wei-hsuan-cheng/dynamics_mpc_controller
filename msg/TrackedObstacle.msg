# One tracked object to keep the robot away from, approximated by spheres in its own frame.
#
# Each TrackedObstacleArray is the complete current set: an id missing from the latest message is
# gone. Keep ids stable while the tracker follows the same object, so it keeps its solver slot.

int32 id

# Object frame in the array's header frame, at header.stamp.
geometry_msgs/Pose pose

# Velocity of the object frame in the header frame (linear at its origin, angular about it).
# The controller extrapolates the pose at constant velocity over the MPC horizon; zero for a
# static object.
geometry_msgs/Twist twist

# Shape: sphere centers in the object frame and their radii [m], the same length.
geometry_msgs/Point[] sphere_centers
float64[] sphere_radii

# 1-sigma position uncertainty [m]; the controller inflates every radius by a multiple of it.
float64 position_std
