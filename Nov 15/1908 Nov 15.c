//************************ Function Declarations //************************
void pause(int seconds);
void Drive(float left, float right, float delaySeconds);
void Drive_PID(float left, float right, float delaySeconds);
void WallFollowing();
void WallMapping();
int CheckBumper();
void Turn(float angle);
int CheckHitWall();
void StraightCruise(int delaySeconds);
//void UpdateCurrentCoord();
void SaveWallCoord(int count);

//************************ Global Constant Declarations //************************
const float PI = 3.14159265359;
const int FRONT_BUMP = 0;
const int BACK_BUMP = 1;//15;
const int LEFT_PHOTO = 3;
const int RIGHT_PHOTO = 4;
const int LEFT_IR = 2;
const int RIGHT_IR = 5;
const int LEFT_MOTOR = 0;
const int RIGHT_MOTOR = 3;

const int TOP_SPEED = 100;

const int LEFT_PHOTO_OFFSET = -27;//110;
//Boolean Constants
//This dialect of C does not have boolean data types, so we"re faking it
//according to convention: true is non-zero, usually 1, and false is 0.
const int TRUE = 1;
const int FALSE = 0;


struct point{
	int x, y;
};

struct current_position{
	int x, y, angle;
};

struct current_position current_pos = {0, 0, 0};
struct point wall_coordinates [4] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};


int main()
{
	pause(1);
  //Turn(4.0 * PI);
	StraightCruise(10);
	return 0;
}

void StraightCruise(int delaySeconds){
	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);

	Drive(0.60, 0.60, delaySeconds);
/*
	int left_motor_count = get_motor_position_counter(LEFT_MOTOR);
	int right_motor_count = get_motor_position_counter(RIGHT_MOTOR);
	int motor_count_avg = (left_motor_count + right_motor_count) / 2.0;
	current_pos.x += sin(current_pos.angle) * motor_count_avg;
	current_pos.y += cos(current_pos.angle) * motor_count_avg;
*/
	printf("L: %d, R: %d.\n", get_motor_position_counter(LEFT_MOTOR), get_motor_position_counter(RIGHT_MOTOR));

	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);
}

void Drive_PID(float left, float right, float delaySeconds){
	delaySeconds *= 1000;

	//set the time of each cycle (i.e. control loop)
	int cycle_msec = 100;

	// get the initial positions of left & right motors.
	int left_motor_pos = get_motor_position_counter(LEFT_MOTOR);
	int right_motor_pos = get_motor_position_counter(RIGHT_MOTOR);


	float gain_p = 0.001; // proportional gain
	float error_p = 0.0; // proportional initialized to 0
	printf("Proportional gain: %f\n", gain_p);

	// variables that store how much left/right motor has turned since the last cycle
	int left_motor_elapse;
	int right_motor_elapse;

	// right motor offset, based on error of the last cycle (error_p) and gain_p
	float right_offset_p = 0.0;

	int count = 0;
	while (count * cycle_msec < delaySeconds){

		motor(LEFT_MOTOR, left * TOP_SPEED);
		motor(RIGHT_MOTOR, (right + right_offset_p) * TOP_SPEED);
		msleep(cycle_msec);
		printf("L_speed: %f, R_speed: %f\n", left * TOP_SPEED, (right + right_offset_p) * TOP_SPEED);

		// get how much left/right wheel has turned.
		left_motor_elapse = get_motor_position_counter(LEFT_MOTOR) - left_motor_pos;
		right_motor_elapse = get_motor_position_counter(RIGHT_MOTOR) - right_motor_pos;

		//update left/right motor positions
		//left_motor_pos = get_motor_position_counter(LEFT_MOTOR);
		//right_motor_pos = get_motor_position_counter(RIGHT_MOTOR);


		// Proportional Error, always = left - right.
		error_p =  left_motor_elapse - right_motor_elapse;

		// compute right motor offset based on feedback from the last cycle
		right_offset_p = gain_p * error_p;

		printf("L_pos: %d, R_pos: %d, err: %f, offset: %f\n", left_motor_elapse, right_motor_elapse, error_p, right_offset_p);
		++count;
	}
}

void Drive(float left, float right, float delaySeconds){
	const float RIGHT_MOTOR_OFFSET = 0.03;
	delaySeconds *= 1000;
	int small_delay_sec = 100;
	int count = 0;
	//test
	int left_motor_pos = get_motor_position_counter(LEFT_MOTOR);
	int right_motor_pos = get_motor_position_counter(RIGHT_MOTOR);
	//
	while (count * small_delay_sec < delaySeconds){
		float adjust_fraction = 1.0 / 5.0;
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

		int left_motor_elapse = get_motor_position_counter(LEFT_MOTOR) - left_motor_pos;
		int right_motor_elapse = get_motor_position_counter(RIGHT_MOTOR) - right_motor_pos;
		left_motor_pos = get_motor_position_counter(LEFT_MOTOR);
		right_motor_pos = get_motor_position_counter(RIGHT_MOTOR);


		printf("L_pos: %d, R_pos: %d\n", left_motor_elapse, right_motor_elapse);

		++count;
}

}

void Turn(float theta){
	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);
	float angle_to_delay_seconds = 0.37;
	float motor_value = 0.5;
	printf("angle: %f rad, delay: %f sec\n", theta, theta * angle_to_delay_seconds);
	// turn left by an angle
	if( theta > 0 ){
		Drive(-motor_value, motor_value, theta * angle_to_delay_seconds);
		ao();
	}
	// turn right by an angle
	else{
		Drive(motor_value, -motor_value, theta * angle_to_delay_seconds);
		ao();
	}
	current_pos.angle += theta;
	printf("Turned!\n");

	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);
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
