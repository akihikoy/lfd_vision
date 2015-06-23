# Ball status kinds (1:in src, 2:in rcv, 3:flow, 4:spill, 5:bounce, 0:unknown)
int32[] ball_st

# Ball positions and velocities [[x,y,z,vx,vy,vz]*N]
float64[] ball_x

# Ball colliding (0: not colliding, 1: colliding, 2: colliding with ground)
int16[] ball_colliding

# Number of balls in each status kind
int32 num_src
int32 num_rcv
int32 num_flow
int32 num_spill
int32 num_bounce

# Height of balls in the receiving container
float64 z_rcv

# Poses (x,y,z,qx,qy,qz,qw) of containers
geometry_msgs/Pose x_src
geometry_msgs/Pose x_rcv

# Position (x,y,z) of pouring point on source container
float64[] p_pour

# Orientation of pouring point
float64 theta

# True if the gripper is in collision
bool gripper_colliding
# If source container is colliding except for balls
bool src_colliding

# Simulation time
float64 time
# True during initializing
bool on_init
