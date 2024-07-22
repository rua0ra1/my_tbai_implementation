# Joint command message
#
# This message contains the name of controlled joint, feed-forward torque,
# desired joint position and velocity together with the kp and kd
# constants for a PD controller.
#
# Applied joint torque is calculated as follows:
# torque = torque_ff + kp * position_error + kd * velocity_error

string joint_name
float64 desired_position
float64 desired_velocity
float64 kp
float64 kd
float64 torque_ff