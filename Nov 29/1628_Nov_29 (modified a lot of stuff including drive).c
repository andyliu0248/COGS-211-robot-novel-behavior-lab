// File: 1443 Nov 17 copy.c
#include <math.h>
#include <time.h>
#include <stdlib.h>

/* --------------------------------------------------------------------------*/
/*                           Function Declarations                           */
/* --------------------------------------------------------------------------*/

// Non-Behavioral Primitives
float random_float(float lower, float upper);
void pause(int seconds);
void initialize_motor_pos();
int corrected_motor_pos(int motor_num);
int ticks_condition(int count, int cycle, int ticks);
int delay_condition(int count, int cycle, int delay);
void update_current_coord_straight(int left_motor_init, int right_motor_init);

// 1st order behaviors
int drive(float left, float right, char condition_choice, int num_ticks_or_delay);
int check_avoid_condition();
int check_front_bumper();
void save_corner_coord(int count);

// 2nd order behaviors
void unstuck();
int turn(float angle);
int straight_cruise(float delaySeconds);

// 3rd order behaviors
void wall_following();
void wall_mapping();
void move_to(int x, int y);
void go_home();


/* --------------------------------------------------------------------------*/
/*                        Global Constant Declarations                       */
/* --------------------------------------------------------------------------*/

// Mathematical Constants
#define PI 3.14159265358979323846
#define FULL_CIRC 2 * PI

// Ports
const int FRONT_BUMP = 0;
const int BACK_BUMP = 1;//15;
const int LEFT_PHOTO = 4;
const int RIGHT_PHOTO = 5;
const int LEFT_IR = 2;
const int RIGHT_IR = 7;
const int LEFT_MOTOR = 0;
const int RIGHT_MOTOR = 3;

// Other Parameters
const int TOP_SPEED = 100;
const int LEFT_PHOTO_OFFSET = -27;//110;
const float RIGHT_MOTOR_OFFSET = 0.02;
const int RIGHT_IR_OFFSET = -70;
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
	int x, y;
	float angle;
};

struct current_position current_pos = {0, 0, 0.0};
struct point wall_coordinates [4] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};

int left_motor_init;
int right_motor_init;

/* --------------------------------------------------------------------------*/
/*                            Function Definitions                           */
/* --------------------------------------------------------------------------*/

int main()
{
	srand(time(NULL));
	initialize_motor_pos();
	straight_cruise(1);
	turn(-1/4.0 * FULL_CIRC);
	straight_cruise(1);
	//unstuck();
	//search_for_food(2);
	//move_to(1000, 3000);
	//printf("go home\n");
	//go_home();
	//go_back_to_last_food();
	//wall_following();
	//turn(1/4.0 * FULL_CIRC);
	return 0;
}

void initialize_motor_pos(){
	left_motor_init = corrected_motor_pos(LEFT_MOTOR);
	right_motor_init = corrected_motor_pos(RIGHT_MOTOR);
}

int corrected_motor_pos(int motor_num){
	int correction_frequency = 50;
	if (motor_num == LEFT_MOTOR){
		return get_motor_position_counter(LEFT_MOTOR);
	}else{
		int right_ticks = get_motor_position_counter(RIGHT_MOTOR);
		//printf("Correction: r: %d -> %d\n", right_ticks, (int)(right_ticks + right_ticks / correction_frequency));
		return (int) (right_ticks + right_ticks / correction_frequency);
	}
}

int ticks_condition(int count, int cycle, int ticks){
	//printf("Checking <ticks>: count: %d, ticks: %d\n", corrected_motor_pos(RIGHT_MOTOR), ticks);
	return abs(corrected_motor_pos(RIGHT_MOTOR) - right_motor_init) < ticks;
}

int delay_condition(int count, int cycle, int delay){
	//printf("Checking <delay>: elapsed: %d, delay: %d\n", count * cycle, delay);
	//return (count * cycle < delay) && check_front_bumper() == FALSE;
	return (count * cycle < delay);
}

int back_condition(int count, int cycle, int delay){
	return (count * cycle < delay);
}

int drive(float left, float right, char condition_choice, int num_ticks_or_delay){

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
	// else if the robot is backing off
	else if (condition_choice == 'b'){
		drive_condition = & back_condition;
	}

	//set the time of each cycle (i.e. PID loop)
	int cycle_msec = 30;

	// get the initial positions of left & right motors.
	//int left_motor_init = corrected_motor_pos(LEFT_MOTOR);
	//int right_motor_init = corrected_motor_pos(RIGHT_MOTOR);
	// moved those lines before main(), making the initial positions of motors global.

	// variables that store how much left/right motor has turned since the last cycle
	int left_motor_ticks = 0;
	int right_motor_ticks = 0;

	// right motor offset, based on error of the last cycle (error_p) and GAIN_P
	float right_offset_p = 0.0;
	int error_p = 0;

	//printf("<Driving_Uni> Proportional gain: %f\n", GAIN_P);

	int count = 0;
	int right_motor_pos_last_cycle = 0;
	int left_motor_pos_last_cycle = 0;
	int right_motor_pos_current;
	int left_motor_pos_current;
	while ((*drive_condition)(count, cycle_msec, num_ticks_or_delay)){
		motor(LEFT_MOTOR, (int) (left * TOP_SPEED));
		motor(RIGHT_MOTOR, (int) ((right + right_offset_p) * TOP_SPEED));
		msleep(cycle_msec);
		right_motor_pos_current = corrected_motor_pos(RIGHT_MOTOR);
		left_motor_pos_current = corrected_motor_pos(LEFT_MOTOR);
		if (abs(right_motor_pos_current - right_motor_pos_last_cycle) < 1 &&
				abs(left_motor_pos_current - left_motor_pos_last_cycle) < 1)
		{
			int left_diff = left_motor_pos_current - left_motor_pos_last_cycle;
			int right_diff = right_motor_pos_current - right_motor_pos_last_cycle;
			printf("motor is dead. dL: %d, dR: %d\n", left_diff, right_diff);
			return -1;
		} else {
			right_motor_pos_last_cycle = right_motor_pos_current;
			left_motor_pos_last_cycle = left_motor_pos_current;
		}
		//printf("L_speed: %d, R_speed: %d\n", (int) (left * TOP_SPEED), (int) ((right + right_offset_p) * TOP_SPEED));

		// get how much left/right wheel has turned.
		left_motor_ticks = corrected_motor_pos(LEFT_MOTOR) - left_motor_init;
		right_motor_ticks = corrected_motor_pos(RIGHT_MOTOR) - right_motor_init;

		// Proportional Error, always = left - right.
		// if straight cruise
		if (left_motor_ticks > 0 && right_motor_ticks > 0){
			error_p = left_motor_ticks - right_motor_ticks;
		}
		// else if backing off
		else if (left_motor_ticks < 0 && right_motor_ticks < 0){
			error_p = right_motor_ticks - left_motor_ticks;
		}
		// else if turning
		else{
			error_p =  - (left_motor_ticks + right_motor_ticks);
		}
		// compute right motor offset based on feedback from the last cycle
		right_offset_p = GAIN_P * error_p;

		//printf("L_pos: %d, R_pos: %d, err: %d, offset: %f\n", left_motor_ticks, right_motor_ticks, error_p, right_offset_p);
		++count;
	}
	if (condition_choice == 'd' || condition_choice == 'b'){
		update_current_coord_straight(left_motor_init, right_motor_init);
	}
	return 1;
}

void update_current_coord_straight(int left_motor_init, int right_motor_init){
	int motor_count_avg = (corrected_motor_pos(LEFT_MOTOR) + corrected_motor_pos(RIGHT_MOTOR)) / 2.0;
	//printf("motor count avg: %d \n", motor_count_avg);
	//ao(); // Stop motors
	current_pos.x += sin(-current_pos.angle) * (motor_count_avg - left_motor_init);
	current_pos.y += cos(-current_pos.angle) * (motor_count_avg - right_motor_init);
	//printf("curent angle: %f, right init: %d\n");
	//printf("x: %d, y: %d\n", current_pos.x, current_pos.y);
}

int straight_cruise(float delaySeconds){
	//printf("Straight Cruise\n");
	//printf("L_pos: %d, R_pos: %d\n", left_motor_init, right_motor_init);
	int success =  drive(0.70, 0.70, 'd', delaySeconds * 1000);
	return success;
	//printf("Cruised! current x: %d, y: %d.\n", current_pos.x, current_pos.y);
}

int turn(float theta){

	//printf("Orientation: %f, ", current_pos.angle);
	float init_angle = current_pos.angle;
	//printf("Angle to be turned: %f, ", theta);

	initialize_motor_pos();

	const float radian_to_ticks = 110; //115; //123
	const float motor_speed = 0.5;

	int success;
	//printf("angle: %f rad, delay: %f sec\n", theta, theta * radian_to_ticks);
	// turn left by an theta
	if( theta > 0 ){
		//printf("%f\n", theta);
		success = drive(-motor_speed, motor_speed, 't', (int)(theta * radian_to_ticks));
		//printf("%f\n", theta);
	}
	// turn right by theta
	else{
		//printf("%f, %d\n", theta, ticks);
		success = drive(motor_speed, -motor_speed, 't', (int)(-theta * radian_to_ticks));
		//printf("%f\n", theta);
	}
	ao(); // Stop motors
	current_pos.angle += theta;
	current_pos.angle = fmod(current_pos.angle, FULL_CIRC);
	//printf("turned %d degrees!\n", (int)(theta/FULL_CIRC*360));
	printf("Turned %f, from %f to randian: %f\n", theta, init_angle, current_pos.angle);

	initialize_motor_pos();
	return success;
}

void unstuck(){
	drive(-0.3, -0.3, 'b', 1000);
}

void search_for_food(int number_of_food_encounters){
	int count = 0;
	struct point last_food_coord = {0,0};
	while (count < number_of_food_encounters){
		if (count > 0){
			unstuck();
			turn(random_float(-FULL_CIRC, FULL_CIRC));
		}
		//int left_motor_activation = 0.6;
		//int right_motor_activation = 0.6;
		while (check_front_bumper() == FALSE && straight_cruise(0.5) > 0) {
		}
		last_food_coord.x = current_pos.x;
		last_food_coord.y = current_pos.y;
		printf("Food #%d at (%d, %d)\n", count, last_food_coord.x, last_food_coord.y);
		count++;
	}
	printf("Food search complete.\n");
	return;
}

/*
def is_success(int success_indicator){
	return (success_indicator > 0? TRUE: FALSE)
}
*/

void move_to(int x, int y){
	float angle_atan = atan((y - current_pos.y) / (x - current_pos.x));
	float angle_to_be_turned = fmod(angle_atan - FULL_CIRC/4 - current_pos.angle, FULL_CIRC);
	int distance_ticks = sqrt(pow(x - current_pos.x, 2) + pow(y - current_pos.y, 2));
	turn(angle_to_be_turned);
	drive(0.6, 0.6, 't', distance_ticks);
}

void go_home(){
	move_to(0,0);
}

void go_back_to_last_food();

float random_float(float lower, float upper)
{
	float interval = upper - lower;
	if (interval < 0){
		interval = -interval;
	}
	float r0_1 = (float)rand()/(float)RAND_MAX;
	return lower + r0_1 * interval;
}

int check_avoid_condition(){
	int left_IR_value;
	int right_IR_value;
	int avoid_threshold = 500;

	left_IR_value = analog_et(LEFT_IR) * 3.36 + 92;
	right_IR_value = analog_et(RIGHT_IR) + RIGHT_IR_OFFSET;

	printf("Left IR: %d, Right IR: %d\n", left_IR_value, right_IR_value);

	if(left_IR_value > avoid_threshold){
		return TRUE;
	}
	else if(right_IR_value > avoid_threshold){
		return TRUE;
	}else{
		return FALSE;
	}
}

int check_front_bumper(){
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

void pause(int seconds){
	int count = 0;
	while (count < seconds){
		msleep(1000);
		count++;
	}
}

void wall_following(){
		straight_cruise(100);
}

void save_corner_coord(int count){
	struct point wall_coordinates [4];
	wall_coordinates[count].x = current_pos.x;
	wall_coordinates[count].y = current_pos.y;
	printf("Save: (%d, %d)\n", wall_coordinates[count].x, wall_coordinates[count].y);
}

void wall_mapping(){
	int count = 0;
	wall_following();
	turn(-1/4.0 * FULL_CIRC);
	while (count < 4){
		wall_following();
		pause(0.5);
		save_corner_coord(count);
		turn(-1/4.0 * FULL_CIRC);
		pause(0.5);
		count++;
	}
	int count_2 = 0;
	while (count_2 < 4){
		struct point wall_coordinates [4];
		printf("n: %d, x: %d, y: %d\n", count_2, wall_coordinates[count].x, wall_coordinates[count].y);
		count_2++;

	}
}
