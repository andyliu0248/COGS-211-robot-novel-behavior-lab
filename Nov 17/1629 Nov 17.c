// File: 1443 Nov 17 copy.c

/* --------------------------------------------------------------------------*/
/*                           Function Declarations                           */
/* --------------------------------------------------------------------------*/

// Non-Behavioral Primitives
void pause(int seconds);
int corrected_motor_pos(int motor_num);
int ticks_condition(int count, int cycle, int ticks);
int delay_condition(int count, int cycle, int delay);

// 1st order behaviors
void Drive_Uni(float left, float right, char condition_choice, int num_ticks_or_delay);
int CheckBumper();
void SaveWallCoord(int count);

// 2nd order behaviors
void Turn(float angle);
void StraightCruise(int delaySeconds);

// 3rd order behaviors
void WallFollowing();
void WallMapping();

// Legacy functions
void brake(float left, float right, int stop_msec, int stop_times);
void Drive_Legacy(float left, float right, float delaySeconds);
void Drive_MRP(int left_ticks, int right_ticks, int left_v, int right_v);
void Drive_PID(float left, float right, float delaySeconds);
void Drive_Ticks(float left, float right, int ticks);


/* --------------------------------------------------------------------------*/
/*                        Global Constant Declarations                       */
/* --------------------------------------------------------------------------*/

// Mathematical Constants
#define PI 3.14159265358979323846

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
const float RIGHT_MOTOR_OFFSET = 0.04;
const float GAIN_P = 0.003; // proportional gain

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
/* ----------------------------------------------------------y----------------*/

int main()
{
	pause(1);
  Turn(4 * 2.0*PI);
	//StraightCruise(5000);
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
		printf("Correction: r: %d -> %d\n", right_ticks, (int)(right_ticks + right_ticks / correction_frequency));
		return (int) (right_ticks + right_ticks / correction_frequency);
}
}

int ticks_condition(int count, int cycle, int ticks){
	//printf("Checking <ticks>: count: %d, ticks: %d\n", corrected_motor_pos(RIGHT_MOTOR), ticks);
	return (corrected_motor_pos(RIGHT_MOTOR) < ticks);
}

int delay_condition(int count, int cycle, int delay){
	//printf("Checking <delay>: elapsed: %d, delay: %d\n", count * cycle, delay);
	return (count * cycle < delay);
}

void Drive_Uni(float left, float right, char condition_choice, int num_ticks_or_delay){

	// define a pointer to the criterion function for while loop
	int (*drive_condition)(int,int,int);

	// if we want to use ticks to control the while loop,
	//    then assign ticks_condition() function to this pointer
	if (condition_choice == 't'){
		drive_condition = & ticks_condition;
	}
	// else if we want to use delay seconds to control the while loop,
	//    then assign delay_condition() function to this pointer
	else if (condition_choice == 'd'){
		drive_condition = & delay_condition;
	}

	//set the time of each cycle (i.e. control loop)
	int cycle_msec = 100;

	// get the initial positions of left & right motors.
	int left_motor_init = corrected_motor_pos(LEFT_MOTOR);
	int right_motor_init = corrected_motor_pos(RIGHT_MOTOR);

	// variables that store how much left/right motor has turned since the last cycle
	int left_motor_ticks = 0;
	int right_motor_ticks = 0;

	// right motor offset, based on error of the last cycle (error_p) and GAIN_P
	float right_offset_p = 0.0;
	int error_p = 0;

	printf("<Driving_Uni> Proportional gain: %f\n", GAIN_P);

	int count = 0;
	while ((*drive_condition)(count, cycle_msec, num_ticks_or_delay)){
		motor(LEFT_MOTOR, (int) (left * TOP_SPEED));
		motor(RIGHT_MOTOR, (int) ((right + right_offset_p) * TOP_SPEED));
		msleep(cycle_msec);
		//printf("L_speed: %d, R_speed: %d\n", (int) (left * TOP_SPEED), (int) ((right + right_offset_p) * TOP_SPEED));

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

void Turn(float theta){

	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);

	const float radian_to_ticks = 123;
	const float motor_speed = 0.5;

	printf("angle: %f rad, delay: %f sec\n", theta, theta * radian_to_ticks);
	// turn left by an theta
	if( theta > 0 ){
		Drive_Uni(-motor_speed, motor_speed, 't', theta * radian_to_ticks);
	}
	// turn right by theta
	else{
		Drive_Uni(motor_speed, -motor_speed, 't', -theta * radian_to_ticks);
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

	Drive_Uni(0.70, 0.70, 'd', delaySeconds * 1000);

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
