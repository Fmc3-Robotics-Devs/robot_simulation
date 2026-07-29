# Precise docking progress (plan section 8.6).
builtin_interfaces/Time stamp

string SEARCH_TAG=SEARCH_TAG
string TAG_VISIBLE=TAG_VISIBLE
string ALIGN_YAW=ALIGN_YAW
string ALIGN_POSITION=ALIGN_POSITION
string FINE_ADJUST=FINE_ADJUST
string SETTLE=SETTLE
string DOCKED=DOCKED
string LOST=LOST

string phase
string station
int32 tag_id
bool tag_visible

# Remaining error in the station frame, metres and radians.
float64 error_x
float64 error_y
float64 error_yaw
# Consecutive frames inside tolerance; docking completes once this reaches the
# configured count, so that a single lucky frame cannot end the approach.
int32 frames_in_tolerance
