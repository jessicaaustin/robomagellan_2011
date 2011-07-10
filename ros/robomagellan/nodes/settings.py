# constants for ROS nodes
#
# all parameters are in SI units (meters, m/s, etc)
#

TEST_MODE = False

#
# robot parameters
#

ROBOT_LENGTH=.64
# maximum speed and turnrate allowed for the robot
MAX_SPEED=1
MAX_TURNRATE=1

#
# General parameters for navigation algorithm
#

# defines a radius around a cone waypoint where we can use short-range sensors
WAYPOINT_DIST=3
# how close we have to be to an intermediate waypoint to consider to have reached it
WAYPOINT_THRESHOLD=.5
# how long to move backwards when backing away from an obstacle
BACKWARDS_TIME=1

# parameters for the point-to-point controller
# these can be tweaked to improve performance
TIMESTEP=.05
# how close we need to get to desired yaw before moving on
THETA_TOLERANCE=.05 #radians
# feedback proportional to longitudinal (in x dir) velocity
LAMBDA=.2
# feedback proportional to lateral (in y dir) error
A1=2
# feedback proportional to angular (theta) error
A2=1

#
# GPS module
#
LAT_OFFSET = 0.0056
LNG_OFFSET = -0.2683

#
# testing
#
COLLISION_THRESHOLD=.6
