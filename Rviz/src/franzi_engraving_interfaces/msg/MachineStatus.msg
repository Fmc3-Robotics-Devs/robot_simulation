# Everything the engraving machine tells the robot (plan section 12.2).
#
# On hardware these are digital inputs or Modbus coils. Nothing here is
# inferred by the robot: if the machine cannot report a signal, the field must
# be driven by something that can, never assumed.
builtin_interfaces/Time stamp

bool machine_ready
bool door_open
bool door_closed
bool clamp_open
bool clamp_closed
bool material_detected
bool machining
bool finished
bool spindle_stopped
bool alarm
bool emergency_stop

# False once the bridge has not heard from the machine within its watchdog
# period. Treat a stale status as unsafe, not as the last known good one.
bool link_ok
