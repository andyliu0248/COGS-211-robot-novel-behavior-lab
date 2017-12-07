// File: 1332_Dec_04.c
#include <math.h>
#include <time.h>
#include <stdlib.h>

/* --------------------------------------------------------------------------*/
/*                           Function Declarations                           */
/* --------------------------------------------------------------------------*/

// vector operations
void cross_product(float a1, float a2, float a3, float b1, float b2, float b3, float* c1, float* c2, float* c3);
float dot_product(float end_x, float end_y, float init_x, float init_y);
int is_to_the_left(float a1, float a2, float a3, float b1, float b2, float b3);
float vector_length(float vec_x, float vec_y);
void vector_normalize(float *x, float *y);
int is_same_direction(float x1, float y1, float x2, float y2);

// ultility functions
int to_degree(float radian);
float random_float(float lower, float upper);

// 0th order behaviors
int corrected_motor_pos(int motor_num);
int ticks_condition(int count, int cycle, int ticks);
int delay_condition(int count, int cycle, int delay);

// 1st order behaviors
void drive(float left, float right, char condition_choice, int num_ticks_or_delay);
int check_front_bumper();

// 2nd order behaviors
void unstuck();
void turn(float angle);
void straight_cruise(float delaySeconds);
void update_current_coord_straight(int left_motor_init, int right_motor_init);


// 3rd order behaviors
void move_to(int x, int y);
void search_for_food(int num_food_encounters);
void go_home();
void visit_food_sites(int num_food_encounters);
void print_food_encounters(int num_food_encounters);

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
const int LEFT_PHOTO_OFFSET = -27; //110;
const float RIGHT_MOTOR_OFFSET = 0.02;
const float GAIN_P = 0.003; // proportional gain

// Boolean Constants
// This dialect of C does not have boolean data types, so we"re faking it
// according to convention: true is non-zero, usually 1, and false is 0.
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

const int num_food = 4;
struct point food_coord [3] = {{0, 0}, {0, 0}, {0, 0}};



/* --------------------------------------------------------------------------*/
/*                            Function Definitions                           */
/* --------------------------------------------------------------------------*/

int main()
{
	// set the random seed
	srand(time(NULL));

	///*
	search_for_food(num_food);
	go_home();
	print_food_encounters(num_food);
	//*/

	return 0;
}

/* --------------------------------------------------------------------------*/
/*                            0th Order Behaviors                            */
/* --------------------------------------------------------------------------*/

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
	return (count * cycle < delay) && check_front_bumper() == FALSE;
}

int back_condition(int count, int cycle, int delay){
	return (count * cycle < delay);
}



/* --------------------------------------------------------------------------*/
/*                            1st Order Behaviors                            */
/* --------------------------------------------------------------------------*/

void drive(float left, float right, char condition_choice, int num_ticks_or_delay){

	// define a pointer to the criterion function for while loop
	int (*drive_condition)(int,int,int);

	/*
	Switch conditions:
		't' = turn by ticks
		'd' = straight cruise, counting by delay seconds
		'x' = straight cruise, counting by ticks
		'b' = retreat, counting by delay seconds
	*/

	// if we want to use ticks to control the while loop,
	//   then assign ticks_condition() function to this pointer
	if (condition_choice == 't' || condition_choice == 'x'){
		drive_condition = & ticks_condition;
	}
	// else if we want to use delay seconds to control the while loop,
	//    then assign delay_condition() function to this pointer
	else if (condition_choice == 'd'){
		drive_condition = & delay_condition;
	// else if the robot is retreating,
	//   
	}else if (condition_choice == 'b'){
		drive_condition = & back_condition;
	}

	//set the time of each cycle (i.e. control loop)
	int cycle_msec = 5;

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
	if (condition_choice == 'd' || condition_choice == 'x' || condition_choice == 'b'){
		update_current_coord_straight(left_motor_init, right_motor_init);
	}
	ao();
	//printf("# cycles completed: %d\n", count);
	msleep(500);
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



/* --------------------------------------------------------------------------*/
/*                            2nd Order Behaviors                            */
/* --------------------------------------------------------------------------*/

void update_current_coord_straight(int left_motor_init, int right_motor_init){
	printf("(%d, %d, %f) => ", current_pos.x, current_pos.y, current_pos.angle);
	int motor_count_avg = (corrected_motor_pos(LEFT_MOTOR) + corrected_motor_pos(RIGHT_MOTOR)) / 2.0;
	//printf("motor count avg: %d \n", motor_count_avg);
	current_pos.x += sin(-current_pos.angle) * (motor_count_avg - left_motor_init);
	current_pos.y += cos(-current_pos.angle) * (motor_count_avg - right_motor_init);
	printf("(%d, %d, %f)\n", current_pos.x, current_pos.y, current_pos.angle);
}

void straight_cruise(float delaySeconds){

	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);

	printf("<straight cruise>\n");
	printf("(%d, %d, %f <=> %d)\n", current_pos.x, current_pos.y, current_pos.angle, to_degree(current_pos.angle));
	drive(0.5, 0.5, 'd', delaySeconds * 1000);
	printf("(%d, %d, %f <=> %d)\n", current_pos.x, current_pos.y, current_pos.angle, to_degree(current_pos.angle));
	printf("<straight cruise> finished.\n\n");

	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);
}

void turn(float theta){

	// if theta is larger than 180, then turn by (360-theta) in the opposite direction
	theta = fmod(theta, FULL_CIRC);
	if (theta > FULL_CIRC / 2){
		theta = - (FULL_CIRC - theta);
	}
	//if -360 < theta < -180, then turn by (360+theta) in the opposite direction
	else if (theta < - FULL_CIRC / 2){
		theta = FULL_CIRC + theta;
	}
	printf("<turn>: %f <=> %d\n", theta, to_degree(theta));
	printf("(%d, %d, %f <=> %d)\n", current_pos.x, current_pos.y, current_pos.angle, to_degree(current_pos.angle));
	msleep(300);
	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);

	const float radian_to_ticks = 115; //123
	const float motor_speed = 0.5;

	//printf("angle: %f rad, delay: %f sec\n", theta, theta * radian_to_ticks);
	// turn left by an theta
	if( theta > 0 ){
		//printf("%f\n", theta);
		drive(-motor_speed, motor_speed, 't', (int)(theta * radian_to_ticks));
		//printf("%f\n", theta);
	}
	// turn right by theta
	else{
		int ticks = -theta * radian_to_ticks;
		//printf("%f, %d\n", theta, ticks);
		drive(motor_speed, -motor_speed, 't', ticks);
		//printf("%f\n", theta);
	}
	ao(); // Stop motors
	current_pos.angle = fmod(current_pos.angle + theta, FULL_CIRC);
	//printf("turned %d degrees!\n", (int)(theta/FULL_CIRC*360));
	printf("(%d, %d, %f <=> %d)\n", current_pos.x, current_pos.y, current_pos.angle, to_degree(current_pos.angle));
	printf("<turn> finished\n\n");
	msleep(300);
	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);
}

void unstuck(){
	msleep(300);
	drive(-0.3, -0.35, 'b', 300);
	msleep(300);
}



/* --------------------------------------------------------------------------*/
/*                            3rd Order Behaviors                            */
/* --------------------------------------------------------------------------*/

void search_for_food(int num_food_encounters){
	int count = 0;
	while (count < num_food_encounters){
		if (count > 0){
			unstuck();
			float angle_to_be_turned;
			//int choose_left = (random_float(-1, 1) > 0)? TRUE: FALSE;
			//float angle_to_be_turned = (choose_left)? FULL_CIRC/4: -FULL_CIRC/4;
			angle_to_be_turned = random_float(-FULL_CIRC/2, FULL_CIRC/2);
			turn(angle_to_be_turned);
		}
		// Straight cruise until robot collides with the wall
		straight_cruise(100);
		// Record this point as a food site coordinate
		struct point food_coord [num_food_encounters];
		food_coord[count].x = current_pos.x;
		food_coord[count].y = current_pos.y;

		count++;
	}
	unstuck();
}

void print_food_encounters(int num_food_encounters){
	struct point food_coord [num_food_encounters];
	int i = 0;
	while (i < num_food_encounters){
		printf("Food #%d: (%d, %d)\n\n", i, food_coord[i].x, food_coord[i].y);
		i++;
	}
}

void go_home(){
	printf("Going home!\n");
	move_to(0,0);
}

void visit_food_sites(int num_food_encounters){
	struct point food_coord [num_food_encounters];
	int i = 0;
	while (i < num_food_encounters){
		move_to(food_coord[i].x, food_coord[i].y);
		i++;
	}
}

void move_to(int x, int y){
	// Angle Calculation
	printf("<<Move to>> (%d, %d, %f) -> (%d, %d)\n", current_pos.x, current_pos.y, current_pos.angle, x, y);
	float dir_x = -sin(current_pos.angle);
	float dir_y = cos(current_pos.angle);
	int target_vec_x = x - current_pos.x;
	int target_vec_y = y - current_pos.y;
	printf("distance vec: (%d, %d)\n", target_vec_x, target_vec_y);

	float dot_p = dot_product(target_vec_x, target_vec_y, dir_x, dir_y);
	float len_p = vector_length(dir_x, dir_y)*vector_length(target_vec_x, target_vec_y);
	float angle_to_be_turned;
	printf("direction vec: (%f, %f)\n", dir_x, dir_y);
	// if target point is not parallel to the direction vector
	if (len_p != 0){
		printf("Not parallel. ");
		float cos_angle_contained =  dot_p / len_p;
		float angle_contained = acos(cos_angle_contained);
		printf("contained: %f\n", angle_contained);
		int left_test = is_to_the_left(dir_x, dir_y, 0, target_vec_x, target_vec_y, 0);
		angle_to_be_turned = (left_test)? angle_contained : -angle_contained;
		printf("left? %d, angle TBT: %f\n", left_test, angle_to_be_turned);
	}
	// if target point is parallel to the direction vector
	// if the same direction
	else if (is_same_direction(dir_x, dir_y, target_vec_x, target_vec_y)){
		angle_to_be_turned = 0;
	// if opposite direction
	}else{
		angle_to_be_turned = FULL_CIRC / 2;
	}
	turn(angle_to_be_turned);

	//distance Calculation
	float distance_gain = 0.9;
	int distance_ticks = distance_gain * pow(pow(x - current_pos.x, 2) + pow(y - current_pos.y,2), 0.5);
	drive(0.6, 0.6, 'x', distance_ticks);
	printf("<<Move to>> finished: (%d, %d, %f)\n\n", current_pos.x, current_pos.y, current_pos.angle);
	return;
}

// To be implemented
//void go_back_to_last_food();

/* --------------------------------------------------------------------------*/
/*                            Ultility Functions                             */
/* --------------------------------------------------------------------------*/

float random_float(float lower, float upper){
	float interval = upper - lower;
	if (interval < 0){
		interval = -interval;
	}
	float r0_1 = (float)rand()/(float)RAND_MAX;
	return lower + r0_1 * interval;
}

int to_degree(float radian){
	return radian / (2 * PI) * 360;
}



/* --------------------------------------------------------------------------*/
/*                            Vector Operations                              */
/* --------------------------------------------------------------------------*/

int is_to_the_left(float a1, float a2, float a3, float b1, float b2, float b3){
	float c1,c2,c3;
	cross_product(b1, b2, b3, a1, a2, a3, &c1, &c2, &c3);
	return c3 < 0;
}

void cross_product(float a1, float a2, float a3, float b1, float b2, float b3, float* c1, float* c2, float* c3){
	*c1 = a2*b3 - a3*b2;
	*c2 = a3*b1 - a1*b3;
	*c3 = a1*b2 - a2*b1;
}

int is_same_direction(float x1, float y1, float x2, float y2){
	float x1_c = x1, y1_c = y1, x2_c = x2, y2_c = y2;
	vector_normalize(&x1_c, &y1_c);
	vector_normalize(&x2_c, &y2_c);
	float new_vec_x = x1_c + x2_c;
	float new_vec_y = y1_c + y2_c;
	float epsilon = 0.001;
	float new_length = vector_length(new_vec_x, new_vec_y);
  float sum_length = vector_length(x1_c, y1_c) + vector_length(x2_c, y2_c);
  printf("new: %f, sum: %f\n, diff: %f\n", new_length, sum_length, fabs(new_length - sum_length));
  return fabs(new_length - sum_length) < epsilon;
}

void vector_normalize(float *x, float *y){
	float length = vector_length(*x, *y);
	*x = *x / length;
	*y = *y / length;
}

float dot_product(float end_x, float end_y, float init_x, float init_y){
	return end_x * init_x + end_y * init_y;
}

float vector_length(float vec_x, float vec_y){
	return pow(pow(vec_x, 2) + pow(vec_y, 2), 0.5);
}
