uint8 UNINITIALIZED=0
uint8 COLLECTING=1
uint8 INITIALIZED=2
uint8 INVALID=3

std_msgs/Header header
uint8 state
uint64 epoch
string odometry_session_id
string alignment_id
geometry_msgs/Transform pool_to_odom
bool output_valid
uint32 sample_count
string reason
