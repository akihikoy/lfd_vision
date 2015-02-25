LabeledPose lpose

# Type of optimization:
#   "xyz"    Optimize x,y,z.
#   "lin2d"  Optimize two parameters on axis1 and axis2.
string type

geometry_msgs/Point axis1
geometry_msgs/Point axis2
geometry_msgs/Point axis3

# Search ranges and number of divisions.
# len(num_div) corresponds with the number of stages.
# For len(num_div)==1: single stage optimization where ranges are:
#   [-range[0]/2,+range[0]/2], [-range[1]/2,+range[1]/2], ...
# For len(num_div)==2: two stage optimization where ranges are:
#   1st stage: [-range[0]/2,+range[0]/2], [-range[1]/2,+range[1]/2], ...
#   2nd stage: [-range[d+0]/2,+range[d+0]/2], [-range[d+1]/2,+range[d+1]/2], ...
#   where d is the number of parameters.
float64[] range
float64[] num_div

# Weights of evaluation function.
# These sizes corresponds with the number of stages.
float64[] weight_depth
float64[] weight_normal
