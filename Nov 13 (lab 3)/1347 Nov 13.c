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
const int LEFT_MOTOR_OFFSET = 0;
//Boolean Constants
//This dialect of C does not have boolean data types, so we're faking it
//according to convention: true is non-zero, usually 1, and false is 0.
const int TRUE = 1;
const int FALSE = 0;

int main()
{
	int i;
	while(1){
    Drive(0.60 + LEFT_MOTOR_OFFSET, 0.60, 0.5);
  }
	return 0;
}

void Drive(float left, float right, float delaySeconds){
	delaySeconds *= 1000;
	motor(LEFT_MOTOR, TOP_SPEED * left);
	motor(RIGHT_MOTOR, TOP_SPEED * right);
	msleep(delaySeconds);
}
