//************************ Function Declarations //************************
void Drive(float left, float right, float delaySeconds);
void WallFollowing();
void WallMapping();
int CheckBumper();
void Turn(float angle);
int CheckHitWall();
void StraightCruise();
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
const float RIGHT_MOTOR_OFFSET = 0.01;
//Boolean Constants
//This dialect of C does not have boolean data types, so we're faking it
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
	prinf("%f", 2.0 * PI)
  Turn(2.0 * PI);
	//msleep(1000);
	return 0;
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

void StraightCruise(){
	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);

	Drive(0.60 + RIGHT_MOTOR_OFFSET, 0.60, 0.5);
	int left_motor_count = get_motor_position_counter(LEFT_MOTOR);
	int right_motor_count = get_motor_position_counter(RIGHT_MOTOR);
	int motor_count_avg = (left_motor_count + right_motor_count) / 2.0;
	current_pos.x += sin(current_pos.angle) * motor_count_avg;
	current_pos.y += cos(current_pos.angle) * motor_count_avg;

	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);
}

/*
int CheckHitWall(){
	int x = current_coordinate.x
	int y = current_coordinate.y
	if()
}
*/

void Turn(float theta){
	clear_motor_position_counter(LEFT_MOTOR);
	clear_motor_position_counter(RIGHT_MOTOR);
	float angle_to_delay_seconds = 0.5;
	float motor_value = 0.5;
	printf("angle: %f, delay: %f\n", theta * angle_to_delay_seconds);
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

void Drive(float left, float right, float delaySeconds){
	delaySeconds *= 1000;
	float adjust_fraction = 3.0 / 4.0;
	float delay_adjust = delaySeconds * adjust_fraction;
	float delay_normal = delaySeconds - delay_adjust;
	motor(LEFT_MOTOR, left * TOP_SPEED);
	motor(RIGHT_MOTOR, right * TOP_SPEED);
	//printf("Hahahahaha");
	//printf("Normal. l: %f, r: %f, delay: %f\n", left, right, delay_normal);
	msleep(delay_normal);

	motor(LEFT_MOTOR, left * TOP_SPEED);
	motor(RIGHT_MOTOR, (right + RIGHT_MOTOR_OFFSET ) * TOP_SPEED);
	//printf("Adjust. l: %f, r: %f, delay: %f\n", left, right, delay_adjust);
	msleep(delay_adjust);
}
