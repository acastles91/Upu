enum class Request {

    auto_reset_request,
    start_moving_forward_request,
    start_moving_backward_request,
    keep_moving_request,
    stopping_request,
    auto_end_request,
    one_frame_request,
    hundred_frames_request,
    loading_request,
    keep_moving_slow_request,
    test_request,
    test_digital_request,
    test_analog_request,
    encoder_test_request,
    calibrate_shutter_request,
    calibrate_sensor_request,
    read_sensor_request,
    print_values_request,
    test_acceleration_request,
    turn_on_interrupt_request,
    turn_off_interrupt_request,
    homing_request,
    test_leds_request,

    //Toggle requests
    //______________________________
    
    direction_toggle_request,
    capture_toggle_request,
    start_stop_toggle_request,
    mode_toggle_request,
    change_speed_toggle_request,
    idle_request,
    set_speed_1_request,
    set_speed_2_request,
    set_speed_3_request,
    set_speed_4_request
    //Test requests
    //_____________________________

    
};