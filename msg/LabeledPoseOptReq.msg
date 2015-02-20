LabeledPose lpose

# Type of optimization.
# Choose from "lin2d", ...
string type

geometry_msgs/Point axis1
geometry_msgs/Point axis2
geometry_msgs/Point axis3

# Search ranges are [-range[0]/2,+range[0]/2], [-range[1]/2,+range[1]/2], ...
float64[] range
float64   num_div

# Weights of evaluation function
float64   weight_depth
float64   weight_normal
