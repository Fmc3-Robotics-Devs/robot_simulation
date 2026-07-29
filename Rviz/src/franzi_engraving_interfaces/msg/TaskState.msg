# Task manager state (plan section 13).
builtin_interfaces/Time stamp

string state
string previous_state
string task_id
int32 retry_count
float64 seconds_in_state
# Empty while healthy; set from ErrorCode when a state fails.
string error_code
string detail
