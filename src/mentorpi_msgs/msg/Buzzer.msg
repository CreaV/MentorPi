# Drives the STM32 piezo buzzer (Function=2 in the RRCLite serial protocol).
# Wire-format on the STM32 side is uint16 LE for all four fields with on/off
# times in *milliseconds*. base_node converts seconds -> ms internally.
uint16  freq      # Hz, 0 silences any in-progress beep
float32 on_time   # seconds, sound duration per cycle
float32 off_time  # seconds, silence between cycles
uint16  repeat    # cycle count, 0 stops, 65535 ~= continuous
