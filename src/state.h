#pragma once


enum class State {

 auto_reset,
 start_moving_forward,
 start_moving_backward,
 is_moving,
 stopping,
 auto_end,
 one_frame,
 hundred_frames,
 loading,
 keep_moving_slow,
 test,
 test_digital,
 test_analog,
 encoder_test,
 calibrate_shutter,
 calibrate_sensor,
 read_sensor,
 print_values,
 test_acceleration,
 direction_toggle,
 turn_on_interrupt,
 turn_off_interrupt,
 toggle_capture,
 change_speed,
 speed_changed,
 idle
};

enum class Mode {

    discrete,
    continuous,
    experimental
};