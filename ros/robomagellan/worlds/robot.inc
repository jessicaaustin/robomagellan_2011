# 
# Defines our robot
#

# IR sensor: Parallx PING))) 
define parallaxping laser
(
  # 2cm to 3m range
  range_min 0.02
  range_max 3.0

  # 40 degrees field of view
  fov 40.0

  # number of laser samples per scan
  samples 10

  # how to display on the robot model
  color "blue"
  size [ 0.02 0.05 0.03 ]
)

# The model for our robot
define robot position
(
  # TODO get accurate size
  size [0.35 0.35 0.25]
  origin [-0.05 0 0 0]
  gui_nose 1

  drive "diff"

  # Can hit things
  obstacle_return 1

  # TODO add other sensors
  # TODO get proper position of lasers
  parallaxping(pose [ 0.150 0.000 -0.150 0.000 ])
)


