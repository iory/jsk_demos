uint8 id
string joint
# Answered its last read.
bool connected
# +1 / -1: raw count increase -> URDF joint angle increase or decrease
int8 direction
# present_position [count], and the joint angle derived from it [rad].
int32 raw_position
float32 angle
# The angle is more than RANGE_MARGIN outside the joint's physical range
# (leader_bus.yaml joint_ranges_deg, else the URDF limits): the
# direction or the calibration of this joint is probably wrong.
bool out_of_range
# Gripper trigger only: mapped from the two recorded counts (released / pulled)
# instead of direction + calibration zero.
bool trigger_calibrated
# Refreshed about once a second.
float32 voltage
int16 temperature
# Failed reads since the driver started, and the last error text.
uint32 read_errors
string last_error
