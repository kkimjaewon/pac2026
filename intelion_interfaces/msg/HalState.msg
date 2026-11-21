std_msgs/Header header
uint8 HAL_INITIALIZING = 0
uint8 HAL_CONNECTING = 1
uint8 HAL_READY = 2
uint8 HAL_ACTIVE = 3
uint8 HAL_FAULT = 4
uint8 HAL_RECONNECTING = 5
uint8 hal_state
uint64 state_seq
int64 received_mono_ns
bool stale
geometry_msgs/Pose tcp_pose
geometry_msgs/Twist tcp_twist
bool pose_valid
bool twist_valid
uint64 active_session_id
uint64 last_command_seq
uint8 CMD_ACCEPTED = 0
uint8 CMD_REJECTED = 1
uint8 CMD_SENT = 2
uint8 last_command_result
uint16[] active_errors
