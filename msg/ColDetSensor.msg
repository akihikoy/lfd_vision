##From color detectors##
#Number of color detectors
int32 num_cols

#Largest color block (contour)#
#Ratio of filled (removed) area to the original area (at reset); [num_cols]
float64[] col_filled_ratio
#Center x,y position; [2*num_cols]
float64[] col_center_xy
#Area; [num_cols]
float64[] col_area
#Bounding box x,y,width,height; [4*num_cols]
float64[] col_bound

#All color blocks (contours)#
#Numbers of blocks [num_cols]
int32[] nums_blocks
#Areas of each block for all num_cols; [sum(nums_blocks)]
float64[] blocks_area
#Center x,y position of each block for all num_cols; [2*sum(nums_blocks)]
float64[] blocks_center_xy


##Flow particles##
#Number of flow particles
int32 num_flows
#Center x,y position of each particle; [2*num_flows]
float64[] flows_xy
#Velocity x,y virtually computed; [2*num_flows]
float64[] flows_vxy
#Speed (magnitude) and direction; [2*num_flows]
float64[] flows_spddir
#Amount of each particle; [num_flows]
float64[] flows_amount

##Flow average##
#Average center x,y position; [2]
float64[] flow_avr_xy
#Average velocity x,y; [2]
float64[] flow_avr_vxy
#Average speed (magnitude) and direction; [2]
float64[] flow_avr_spddir
