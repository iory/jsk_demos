Header header

# Where commands go: "virtual" or "real".
string target
# True when the "real" target is the simulated stand-in started by
# teleop.launch real:=false, not hardware.
bool real_is_simulated

# idle / approaching / following / moving_to_zero / leader_lost / error
string phase
string message

string[] arms
bool[] arms_enabled

# "feetech" or "gui"
string leader_source
bool leader_alive
bool leader_calibrated
float32 leader_rate
# Largest |command - follower state| over the enabled joints [rad or m].
float32 max_tracking_error

bool recording
string record_directory
string current_bag
float32 recording_duration
uint64 current_bag_size
uint32 bag_count
uint64 total_bag_size
uint64 disk_free
