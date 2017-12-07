/* --------------------------------------------------------------------------*/
/*                           Function Declarations                           */
/* --------------------------------------------------------------------------*/

// Non-Behavioral Primitives
void pause(int seconds);
int corrected_motor_pos(int motor_num);
void SaveWallCoord(int count);

// Behavioral Primitives
void brake(float left, float right, int stop_msec, int stop_times);
void Drive_Legacy(float left, float right, float delaySeconds);
void Drive_PID(float left, float right, float delaySeconds);
void Drive_MRP(int left_ticks, int right_ticks, int left_v, int right_v);
int CheckBumper();

// Higher-Order Behaviors
void Turn(float angle);
void StraightCruise(int delaySeconds);
void WallFollowing();
void WallMapping();



/* --------------------------------------------------------------------------*/
/*                        Global Constant Declarations                       */
/* --------------------------------------------------------------------------*/

// Mathematical Constants
const float PI = 3.14159265359;

// Ports
const int FRONT_BUMP = 0;
const int BACK_BUMP = 1;//15;
const int LEFT_PHOTO = 3;
const int RIGHT_PHOTO = 4;
const int LEFT_IR = 2;
const int RIGHT_IR = 5;
const int LEFT_MOTOR = 0;
const int RIGHT_MOTOR = 3;

// Other Parameters
const int TOP_SPEED = 100;
const int LEFT_PHOTO_OFFSET = -27;//110;
const float RIGHT_MOTOR_OFFSET = 0.03;

//Boolean Constants
//This dialect of C does not have boolean data types, so we"re faking it
//according to convention: true is non-zero, usually 1, and false is 0.
const int TRUE = 1;
const int FALSE = 0;



/* --------------------------------------------------------------------------*/
/*                                 Structures                                */
/* --------------------------------------------------------------------------*/

struct point{
	int x, y;
};

struct current_position{
	int x, y, angle;
};

struct current_position current_pos = {0, 0, 0};
struct point wall_coordinates [4] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};



/* --------------------------------------------------------------------------*/
/*                            Function Definitions                           */
/* --------------------------------------------------------------------------*/

int main()
{
	pause(1);
  Turn(5 * 2.0 * PI);
	//StraightCruise(5);
	return 0;
}

void pause(int seconds){
	//printf("Pausing %d sec", seconds);
	int count = 0;
	while (count < seconds){
		msleep(1000);
		//printf("-->");
		count++;
	}
	//printf("Resume!\n");
}


int corrected_motor_pos(int motor_num){
	int correction_frequency = 50;
	if (motor_num == LEFT_MOTOR){
		return get_motor_position_counter(LEFT_MOTOR);
	}else{
		int right_ticks = get_motor_position_counter(RIGHT_MOTOR);
		printf("r: %d -> %d\n", right_ticks, (int)(right_ticks + right_ticks / correction_frequency));
		return (int) (right_ticks + right_ticks / correction_frequency);
}
}


void Drive_PID(float left, float right, float delaySeconds){
	delaySeconds *= 1000;

	//set the time of each cycle (i.e. control loop)
	int cycle_msec = 100;

	// get the initial positions of left & right motors.
	int left_motor_init = corrected_motor_pos(LEFT_MOTOR);
	int right_motor_init = corrected_motor_pos(RIGHT_MOTOR);

	float gain_p = 0.001; // proportional gain
	printf("Proportional gain: %f\n", gain_p);

	// variables that store how much left/right motor has turned since the last cycle
	int left_motor_ticks = 0;
	int right_motor_ticks = 0;

	// right motor offset, based on error of the last cycle (error_p) and gain_p
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
		right_offset_p = gain_p * error_p;

		printf("L_pos: %d, R_pos: %d, err: %d, offset: %f\n", left_motor_ticks, right_motor_ticks, error_p, right_offset_p);
		++count;
	}
}


void Turn(float theta){

	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);

	const float angle_to_delay_seconds = 0.368;
	const float motor_speed = 0.5;

	printf("angle: %f rad, delay: %f sec\n", theta, theta * angle_to_delay_seconds);
	// turn left by an theta
	if( theta > 0 ){
		Drive_PID(-motor_speed, motor_speed, theta * angle_to_delay_seconds);
	}
	// turn right by theta
	else{
		Drive_PID(motor_speed, -motor_speed, -theta * angle_to_delay_seconds);
	}
	ao(); // Stop motors
	current_pos.angle += theta;
	printf("Turned %d degrees!\n", (int)(theta/(2*PI)*360));

	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);
}


void StraightCruise(int delaySeconds){

	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);

	Drive_PID(0.70, 0.70, delaySeconds);

	// Update current coord
/*
	int left_motor_count = get_motor_position_counter(LEFT_MOTOR);
	int right_motor_count = get_motor_position_counter(RIGHT_MOTOR);
	int motor_count_avg = (left_motor_count + right_motor_count) / 2.0;
	current_pos.x += sin(current_pos.angle) * motor_count_avg;
	current_pos.y += cos(current_pos.angle) * motor_count_avg;
*/
	//testing
	//printf("L: %d, R: %d.\n", get_motor_position_counter(LEFT_MOTOR), get_motor_position_counter(RIGHT_MOTOR));
	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);
}


int CheckBumper(){
	int bumpThreshold = 250;
	int bumpMax = 400;
	int frontBumpValue = analog10(FRONT_BUMP);

	if(frontBumpValue < bumpThreshold){
		return TRUE;
	}
	else if(frontBumpValue >= bumpThreshold && frontBumpValue <= bumpMax){
		return TRUE;
	}else{
		return FALSE;
	}
}


void WallFollowing(){
	int leftIRValue;
	int rightIRValue;
	int avoidThreshold = 500;

	leftIRValue = analog_et(LEFT_IR);
	rightIRValue = analog_et(RIGHT_IR);
	while (TRUE) {
		if (CheckBumper() == TRUE){
			if (leftIRValue - rightIRValue < 10){
				return;
			}else{
				Turn(10);
				StraightCruise(10);
				// this should be arc cruise ?
			}
		}
	}
}


void SaveWallCoord(int count){
	struct point wall_coordinates [4];
	wall_coordinates[count].x = current_pos.x;
	wall_coordinates[count].y = current_pos.y;
}


void WallMapping(){
	int count = 0;
	while (count < 4){
		WallFollowing();
		SaveWallCoord(count);
		Turn(-90);
		count++;
	}
}
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
		//printf("Hahahahaha");
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
