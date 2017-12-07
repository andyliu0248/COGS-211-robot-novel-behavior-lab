/* --------------------------------------------------------------------------*/
/*                               Legacy Function                             */
/* --------------------------------------------------------------------------*/

void Drive_Legacy(float left, float right, float delaySeconds){

	delaySeconds *= 1000;
	int small_delay_sec = 100;
	int count = 0;
	//test
	int left_motor_pos = corrected_motor_pos(LEFT_MOTOR);
	int right_motor_pos = corrected_motor_pos(RIGHT_MOTOR);
	//
	while (count * small_delay_sec < delaySeconds){
		float adjust_fraction = 1.0 / 3.0;
		float delay_adjust = small_delay_sec * adjust_fraction;
		float delay_normal = small_delay_sec - delay_adjust;
		motor(LEFT_MOTOR, left * TOP_SPEED);
		motor(RIGHT_MOTOR, right * TOP_SPEED);

		printf("Normal. l: %04f, r: %04f, delay: %04f\n", left, right, delay_normal);
		msleep(delay_normal);

		motor(LEFT_MOTOR, left * TOP_SPEED);
		motor(RIGHT_MOTOR, (right + RIGHT_MOTOR_OFFSET ) * TOP_SPEED);
		printf("Adjust. l: %04f, r: %04f, delay: %04f\n", left, right+RIGHT_MOTOR_OFFSET, delay_adjust);
		msleep(delay_adjust);

		int left_motor_ticks = get_motor_position_counter(LEFT_MOTOR) - left_motor_pos;
		int right_motor_ticks = get_motor_position_counter(RIGHT_MOTOR) - right_motor_pos;
		left_motor_pos = get_motor_position_counter(LEFT_MOTOR);
		right_motor_pos = get_motor_position_counter(RIGHT_MOTOR);

		printf("L_pos: %d, R_pos: %d\n", left_motor_ticks, right_motor_ticks);
		++count;
	}
}


void Drive_MRP(int left_ticks, int right_ticks, int left_v, int right_v){
	left_v *= 1000;
	right_v *= 1000;
	mrp(LEFT_MOTOR, left_v, left_ticks);
	mrp(RIGHT_MOTOR, right_v, right_ticks);
}

void Drive_PID(float left, float right, float delaySeconds){
	delaySeconds *= 1000;

	//set the time of each cycle (i.e. control loop)
	int cycle_msec = 100;

	// get the initial positions of left & right motors.
	int left_motor_init = corrected_motor_pos(LEFT_MOTOR);
	int right_motor_init = corrected_motor_pos(RIGHT_MOTOR);


	printf("Proportional gain: %f\n", GAIN_P);

	// variables that store how much left/right motor has turned since the last cycle
	int left_motor_ticks = 0;
	int right_motor_ticks = 0;

	// right motor offset, based on error of the last cycle (error_p) and GAIN_P
	float right_offset_p = 0.0;
	int error_p = 0;

	int count = 0;
	while (count * cycle_msec < delaySeconds){

		motor(LEFT_MOTOR, (int) (left * TOP_SPEED));
		motor(RIGHT_MOTOR, (int) ((right + right_offset_p) * TOP_SPEED));
		msleep(cycle_msec);
		printf("L_speed: %d, R_speed: %d\n\n", (int) (left * TOP_SPEED), (int) ((right + right_offset_p) * TOP_SPEED));

		// get how much left/right wheel has turned.
		left_motor_ticks = corrected_motor_pos(LEFT_MOTOR) - left_motor_init;
		right_motor_ticks = corrected_motor_pos(RIGHT_MOTOR) - right_motor_init;

		// Proportional Error, always = left - right.
		// if straight cruise
		if (left_motor_ticks > 0 && right_motor_ticks > 0){
			error_p =  left_motor_ticks - right_motor_ticks;
		}
		// else if turning
		else{
			error_p =  - (left_motor_ticks + right_motor_ticks);
		}
		// compute right motor offset based on feedback from the last cycle
		right_offset_p = GAIN_P * error_p;

		printf("L_pos: %d, R_pos: %d, err: %d, offset: %f\n", left_motor_ticks, right_motor_ticks, error_p, right_offset_p);
		++count;
	}
}

void Drive_Ticks(float left, float right, int ticks){
	//set the time of each cycle (i.e. control loop)
	int cycle_msec = 100;

	// get the initial positions of left & right motors.
	int left_motor_init = corrected_motor_pos(LEFT_MOTOR);
	int right_motor_init = corrected_motor_pos(RIGHT_MOTOR);


	printf("Proportional gain: %f\n", GAIN_P);

	// variables that store how much left/right motor has turned since the last cycle
	int left_motor_ticks = 0;
	int right_motor_ticks = 0;

	// right motor offset, based on error of the last cycle (error_p) and GAIN_P
	float right_offset_p = 0.0;
	int error_p = 0;

	int count = 0;
	while (abs(corrected_motor_pos(RIGHT_MOTOR)) < ticks){

		motor(LEFT_MOTOR, (int) (left * TOP_SPEED));
		motor(RIGHT_MOTOR, (int) ((right + right_offset_p) * TOP_SPEED));
		msleep(cycle_msec);
		printf("L_speed: %d, R_speed: %d\n\n", (int) (left * TOP_SPEED), (int) ((right + right_offset_p) * TOP_SPEED));

		// get how much left/right wheel has turned.
		left_motor_ticks = corrected_motor_pos(LEFT_MOTOR) - left_motor_init;
		right_motor_ticks = corrected_motor_pos(RIGHT_MOTOR) - right_motor_init;

		// Proportional Error, always = left - right.
		// if straight cruise
		if (left_motor_ticks > 0 && right_motor_ticks > 0){
			error_p =  left_motor_ticks - right_motor_ticks;
		}
		// else if turning
		else{
			error_p =  - (left_motor_ticks + right_motor_ticks);
		}
		// compute right motor offset based on feedback from the last cycle
		right_offset_p = GAIN_P * error_p;

		printf("L_pos: %d, R_pos: %d, err: %d, offset: %f\n", left_motor_ticks, right_motor_ticks, error_p, right_offset_p);
		++count;
	}
}


void brake(float left, float right, int stop_msec, int stop_times){
	left *= 0.9;
	right *= 0.9;
	int sleep_interval = stop_msec / stop_times;
	int count = 0;
	while (count * sleep_interval < stop_msec){
		motor(RIGHT_MOTOR, right * TOP_SPEED);
		motor(LEFT_MOTOR, left * TOP_SPEED);
		left *= 0.5;
		right *= 0.5;
		msleep(sleep_interval);
		count++;
	}
}
