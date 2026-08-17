std_msgs/Header header
string mission_id
string plan_hash
string contract_version
string canonical_plan_json
uint64 localization_epoch
string odometry_session_id
string alignment_id
geometry_msgs/Point[] waypoints
float32 cruise_speed
# One value per waypoint (v1 contract only), indexed by the leg *departing*
# that waypoint toward the next. Empty means every leg uses cruise_speed
# above, unchanged from before this field existed.
float32[] cruise_speed_per_leg
float32 lookahead_dist
float32 reach_threshold
string heading_mode
bool loop
