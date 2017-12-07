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
int corrected_motor_pos(int motor_num);
int ticks_condition(int count, int cycle, int ticks);
int delay_condition(int count, int cycle, int delay);
void update_current_coord_straight(int left_motor_init, int right_motor_init);

// 1st order behaviors
void Drive_Uni(float left, float right, char condition_choice, int num_ticks_or_delay);
int CheckFrontBumper();
void SaveCornerCoord(int count);

// 2nd order behaviors
void Unstuck();
void Turn(float angle);
void StraightCruise(float delaySeconds);

// 3rd order behaviors
void WallFollowing();
void WallMapping();
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
const int LEFT_PHOTO = 3;
const int RIGHT_PHOTO = 4;
const int LEFT_IR = 2;
const int RIGHT_IR = 5;
const int LEFT_MOTOR = 0;
const int RIGHT_MOTOR = 3;

// Other Parameters
const int TOP_SPEED = 100;
const int LEFT_PHOTO_OFFSET = -27;//110;
const float RIGHT_MOTOR_OFFSET = 0.02;
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



/* --------------------------------------------------------------------------*/
/*                            Function Definitions                           */
/* --------------------------------------------------------------------------*/

int main()
{
	srand(time(NULL));
	//Unstuck();
	//search_for_food(1);
	move_to(100, 300);
	move_to(0, 0);
	//go_home();
	//go_back_to_last_food();
	//WallFollowing();
	//StraightCruise(5);
	//Turn(-1/4.0 * FULL_CIRC);
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
		//printf("Correction: r: %d -> %d\n", right_ticks, (int)(right_ticks + right_ticks / correction_frequency));
		return (int) (right_ticks + right_ticks / correction_frequency);
	}
}

int ticks_condition(int count, int cycle, int ticks){
	//printf("Checking <ticks>: count: %d, ticks: %d\n", corrected_motor_pos(RIGHT_MOTOR), ticks);
	return abs(corrected_motor_pos(RIGHT_MOTOR)) < ticks;
}

int delay_condition(int count, int cycle, int delay){
	//printf("Checking <delay>: elapsed: %d, delay: %d\n", count * cycle, delay);
	return (count * cycle < delay) && CheckFrontBumper() == FALSE;
}

int back_condition(int count, int cycle, int delay){
	return (count * cycle < delay);
}

void Drive_Uni(float left, float right, char condition_choice, int num_ticks_or_delay){

	// define a pointer to the criterion function for while loop
	int (*drive_condition)(int,int,int);

	// if we want to use ticks to control the while loop,
	//    then assign ticks_condition() function to this pointer
	if (condition_choice == 't' || condition_choice == 'x'){
		drive_condition = & ticks_condition;
	}
	// else if we want to use delay seconds to control the while loop,
	//    then assign delay_condition() function to this pointer
	else if (condition_choice == 'd'){
		drive_condition = & delay_condition;
	}else if (condition_choice == 'b'){
		drive_condition = & back_condition;
	}

	//set the time of each cycle (i.e. control loop)
	int cycle_msec = 20;

	// get the initial positions of left & right motors.
	int left_motor_init = corrected_motor_pos(LEFT_MOTOR);
	int right_motor_init = corrected_motor_pos(RIGHT_MOTOR);

	// variables that store how much left/right motor has turned since the last cycle
	int left_motor_ticks = 0;
	int right_motor_ticks = 0;

	// right motor offset, based on error of the last cycle (error_p) and GAIN_P
	float right_offset_p = 0.0;
	int error_p = 0;

	//printf("<Driving_Uni> Proportional gain: %f\n", GAIN_P);

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
			error_p = left_motor_ticks - right_motor_ticks;
		}
		//else if backing off
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
	if (condition_choice == 'd' || condition_choice == 'x'){
		update_current_coord_straight(left_motor_init, right_motor_init);
	}
}

void update_current_coord_straight(int left_motor_init, int right_motor_init){
	int motor_count_avg = (corrected_motor_pos(LEFT_MOTOR) + corrected_motor_pos(RIGHT_MOTOR)) / 2.0;
	printf("motor count avg: %d \n", motor_count_avg);
	//ao(); // Stop motors
	current_pos.x += sin(-current_pos.angle) * (motor_count_avg - left_motor_init);
	current_pos.y += cos(-current_pos.angle) * (motor_count_avg - right_motor_init);
	printf("curent angle: %f, right init: %d\n");
	printf("x: %d, y: %d\n", current_pos.x, current_pos.y);
}

void Turn(float theta){

	printf("Orientation: %f\n", current_pos.angle);
	printf("Angle to be turned: %f\n", theta);

	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);

	const float radian_to_ticks = 115; //123
	const float motor_speed = 0.5;

	//printf("angle: %f rad, delay: %f sec\n", theta, theta * radian_to_ticks);
	// turn left by an theta
	if( theta > 0 ){
		//printf("%f\n", theta);
		Drive_Uni(-motor_speed, motor_speed, 't', (int)(theta * radian_to_ticks));
		//printf("%f\n", theta);
	}
	// turn right by theta
	else{
		int ticks = -theta * radian_to_ticks;
		//printf("%f, %d\n", theta, ticks);
		Drive_Uni(motor_speed, -motor_speed, 't', ticks);
		//printf("%f\n", theta);
	}
	ao(); // Stop motors
	current_pos.angle += theta;
	current_pos.angle = fmod(current_pos.angle, FULL_CIRC);
	//printf("Turned %d degrees!\n", (int)(theta/FULL_CIRC*360));
	printf("Turned! Current randian: %f, degree: %f\n", current_pos.angle, current_pos.angle/FULL_CIRC*360.0);

	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);
}

void Unstuck(){
	Drive_Uni(-0.3, -0.3, 'b', 1000);
}

void StraightCruise(float delaySeconds){

	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);

	printf("Straight Cruise\n");
	Drive_Uni(0.70, 0.70, 'd', delaySeconds * 1000);
	printf("Cruised! current x: %d, y: %d.\n", current_pos.x, current_pos.y);

	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);
}

int CheckFrontBumper(){
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
		StraightCruise(100);
}

void SaveCornerCoord(int count){
	struct point wall_coordinates [4];
	wall_coordinates[count].x = current_pos.x;
	wall_coordinates[count].y = current_pos.y;
	printf("Save: (%d, %d)\n", wall_coordinates[count].x, wall_coordinates[count].y);
}

void search_for_food(int number_of_food_encounters){
	int count = 0;
	struct point last_food_coord = {0,0};
	while (count < number_of_food_encounters){
		if (count > 0){
			Unstuck();
			Turn(random_float(-FULL_CIRC, FULL_CIRC));
		}
		StraightCruise(100);
		last_food_coord.x = current_pos.x;
		last_food_coord.y = current_pos.y;
		count++;
	}
}

void move_to(int x, int y){
	printf("Moving to (%d, %d)\n", x, y);
	printf("Current pos (%d, %d, %f)\n", current_pos.x, current_pos.y, current_pos.angle);
	float angle_atan = atan((y - current_pos.y) / (x - current_pos.x));
	printf("ATAN: %f\n", angle_atan);
	float angle_to_be_turned = fmod(angle_atan + FULL_CIRC/4 - current_pos.angle, FULL_CIRC);
	int distance_ticks = sqrt(pow(x - current_pos.x, 2) + pow(y - current_pos.y, 2));
	Turn(angle_to_be_turned);
	Drive_Uni(0.6, 0.6, 'x', distance_ticks);
}

void go_home(){
	printf("Going home!\n");
	move_to(0,0);
}
//go_back_to_last_food();

float random_float(float lower, float upper)
{
	float interval = upper - lower;
	if (interval < 0){
		interval = -interval;
	}
	float r0_1 = (float)rand()/(float)RAND_MAX;
	return lower + r0_1 * interval;
}


void WallMapping(){
	int count = 0;
	WallFollowing();
	Turn(-1/4.0 * FULL_CIRC);
	while (count < 4){
		WallFollowing();
		pause(0.5);
		SaveCornerCoord(count);
		Turn(-1/4.0 * FULL_CIRC);
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
