//************************ Function Declarations //************************
void Drive(float left, float right, float delaySeconds);


//************************ Global Constant Declarations //************************
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

int main()
{
	int i;
	while(1){
		//printf("Hahahahaha");
    Drive(0.60, 0.60, 0.5);
  }
	return 0;
}

void Drive(float left, float right, float delaySeconds){
	delaySeconds *= 1000;
	float adjust_fraction = 3.0 / 4.0;
	float delay_adjust = delaySeconds * adjust_fraction;
	float delay_normal = delaySeconds - delay_adjust;
	motor(LEFT_MOTOR, left * TOP_SPEED);
	motor(RIGHT_MOTOR, right * TOP_SPEED);
	//printf("Hahahahaha");
	printf("Normal. l: %f, r: %f, delay: %f\n", left, right, delay_normal);
	msleep(delay_normal);

	motor(LEFT_MOTOR, left * TOP_SPEED);
	motor(RIGHT_MOTOR, (right + RIGHT_MOTOR_OFFSET ) * TOP_SPEED);
	printf("Adjust. l: %f, r: %f, delay: %f\n", left, right, delay_adjust);
	msleep(delay_adjust);
}
