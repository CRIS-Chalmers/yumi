std_msgs/Header header
#"Left" or "Right"
string arm

float64 max_linear_vel
float64 max_angular_vel

#values between -1 and 1 
geometry_msgs/Vector3 deltaP
geometry_msgs/Vector3 deltaO

# values between -1 (close) and 1 (open)
float64 gripper
