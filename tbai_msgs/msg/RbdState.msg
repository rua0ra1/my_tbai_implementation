# Robot state message
# The message contains the following values:
# Base orientation - ZYX Euler angles - 3
# Base position in world frame - 3
# Base angular velocity in base frame - 3
# Base linear velocity in base frame - 3
# Joint angles - 12
# Joint velocities - 12
#
# In total, there are (3+3+3+3+12+12) = 36 values in the message.
# The message also includes contact information for the feet.
# There are four feet, each contains a single contact flag
# The feet are ordered as follows: LF, RF, LH, RH

time stamp
float64[36] rbd_state
bool[4] contact_flags