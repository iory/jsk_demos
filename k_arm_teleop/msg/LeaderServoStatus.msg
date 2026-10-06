Header header
string port
# Bus-level problem (port missing, could not open), empty when the bus is open.
string bus_error
bool calibrated
# joint_states is published only while every servo is connected.
bool all_connected
ServoState[] servos
