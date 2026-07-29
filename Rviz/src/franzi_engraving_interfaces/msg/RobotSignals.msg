# Everything the robot tells the machine (plan section 12.3).
builtin_interfaces/Time stamp

bool load_request
bool unload_request
bool start_request
# The arm is fully outside the machine envelope. This is the interlock the
# machine relies on before it may move anything.
bool robot_clear
bool robot_busy
bool robot_fault
bool reset_request
