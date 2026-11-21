uint8 CMD_POSITION = 0
uint8 CMD_FORCE_ADJUST = 1
uint8 CMD_PATH_ADJUST = 2
uint8 ST_OK = 0
uint8 ST_NOT_DETECTED = 1
uint8 ST_SENSOR_INVALID = 2
uint8 ST_UNKNOWN = 3

uint64 seq
uint32 writer_boot_id
uint16 layout_version
uint8 command_type
uint8 status
float64[4] target
float64 confidence
int64 age_ns
