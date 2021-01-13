def start_mode_2( pos_Kp, pos_Kd, ramp_rate ):

    target_pos = 8  #The target position is 8 ft infront of the runner
    Base_speed = 60  #May change this value later.
    odrv0.axis0.controller.config.vel_ramp_rate = ramp_rate
    odrv0.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
    odrv0.axis0.requested_state = AXIS_STATE_SENSORLESS_CONTROL

    direction = 1
    last_error = 0
    while True:
        distance = rotate_lidar(direction) #"distance" is set equal to the closest value from the sweep
           
        if direction == 1: #switch the "direction" variable so that the sweep cycle switches direction
            direction = 0
        else:
            direction = 1

        if distance != 100:
            pos_error = distance - target_pos #set "pos_error" equal to the difference between the target and actual position
            #calculate motor_adjustment with PID equation
            motor_adjustment = (pos_Kp*pos_error) + (pos_Kd*(pos_error - last_error))
            motor_adjustment = motor_adjustment * (-1)
            if motor_adjustment > 40: #max out so robot can't exceed a speed of 100. I will remove this later
                motor_adjustment = 40
            if motor_adjustment < -60: #minimum speed adjustment of -60 so robot can't go backwards
                motor_adjustment = -60
            odrv0.axis0.controller.input_vel = Base_speed + motor_adjustment #adjust the input_vel
            last_error = pos_error #re-set last error for the next cycle of the PID equation
        if keyboard.is_pressed('q'):
            return
