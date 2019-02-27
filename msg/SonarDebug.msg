# Sonar debug message
# Contains full sonar data for debugging and plotting

Header header

# raw signals
float32[] h_a
float32[] h_refa
float32[] h_b
float32[] h_refb

# send fft
float32[] fft_freq
float32[] fft_mag

# send signal stats
float32 actual_freq
float32 shift_a
float32 shift_b
float32 rel_heading
float32 abs_heading

# robot pose
geometry_msgs/Pose2D robot_pose

# particle filter
bool filter_running
float32[] particles_x
float32[] particles_y

# filter output
float32[2] pinger_location
float32[4] filter_cov