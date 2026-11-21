# C-internal receiver adoption ACK, not a TCP arrival/completion ACK.
uint64 session_id
uint64 last_applied_command_seq
int64 applied_mono_ns
int64 sampled_mono_ns
bool active
bool watchdog_active
bool rejected
uint8 joint_count
float64[7] commanded_positions
float64[7] commanded_velocities
