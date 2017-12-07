// Created Nick Livingston 2017

//*************************************************** Function Declarations ***********************************************************//
//Behavior Primitive Functions
int CheckFrontBumpValueConditions();
int CheckBackBumpValueConditions();
int CheckSeekLightConditions();
int CheckSeekDarkConditions();
int CheckAvoidConditions();
int CheckApproachConditions();
void EscapeFrontBump();
void EscapeBackBump();
void SeekLight();
void SeekDark();
void Avoid();
void Approach();
void StraightCruise();
void StraightBack();
void ArcCruise();
int min(int first, int second);
int max(int first, int second);
//Motor Control Function
void Drive(float left, float right, float delaySeconds);

//*************************************************** Global Constant Declarations ****************************************************//
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
const int LEFT_MOTOR_OFFSET = 0.3;
//Boolean Constants
//This dialect of C does not have boolean data types, so we're faking it
//according to convention: true is non-zero, usually 1, and false is 0.
const int TRUE = 1;
const int FALSE = 0;

//*************************************************** Function Definitions ****************************************************//
int main()
{
	int i;
	while(1){
		if(CheckFrontBumpValueConditions() == TRUE){
            EscapeFrontBump();
        }
        else if(CheckBackBumpValueConditions() == TRUE){
            EscapeBackBump();
        }
        else if(CheckAvoidConditions() == TRUE){
            Avoid();
        }
        else
				if(CheckSeekLightConditions() == TRUE){
            SeekLight();
        }
        else{
          StraightCruise();
        }
	}
	return 0;
}

/******************************************************/
int CheckFrontBumpValueConditions(){
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

/******************************************************/
int CheckBackBumpValueConditions(){
	int bumpThreshold = 250;
	int bumpMax = 400;

	int backBumpValue = analog10(BACK_BUMP);

	if(backBumpValue < bumpThreshold){
		return TRUE;
	}
	else if(backBumpValue >= bumpThreshold && backBumpValue <= bumpMax)
	{
		return TRUE;
	}else{
		return FALSE;
	}
}

/******************************************************/
int CheckSeekLightConditions(){
	int leftPhotoValue;
	int rightPhotoValue;
	int photoDifference;
	int photoThreshold = 10;
	int minSeek   = 1023 * 0.45;

	leftPhotoValue = 1023 - analog10(LEFT_PHOTO) + LEFT_PHOTO_OFFSET;
	rightPhotoValue = 1023 - analog10(RIGHT_PHOTO);
	photoDifference = leftPhotoValue - rightPhotoValue;
	printf("SeekLight Cond...?\n");
	printf("lpv: %d, rpv: %d, diff: %d\n", leftPhotoValue, rightPhotoValue, photoDifference);
	if(photoDifference > photoThreshold){// && min(leftPhotoValue, rightPhotoValue) > minSeek){
		printf("lpv: %d, rpv: %d, diff: %d\n", leftPhotoValue, rightPhotoValue, photoDifference);
		return TRUE;
	}
	else if(photoDifference < -photoThreshold){// && min(leftPhotoValue, rightPhotoValue) > minSeek){
		printf("lpv: %d, rpv: %d, diff: %d\n", leftPhotoValue, rightPhotoValue, photoDifference);
		return TRUE;
	}
	else{
		return FALSE;
	}
}

/******************************************************/
int CheckSeekDarkConditions(){
	int leftPhotoValue;
	int rightPhotoValue;
	int photoDifference;
	int photoThreshold = 30;
	int maxSeek   = 1023 * 0.40;

	leftPhotoValue = 1023 - analog10(LEFT_PHOTO);
	rightPhotoValue = 1023 - analog10(RIGHT_PHOTO);
	photoDifference = leftPhotoValue - rightPhotoValue;

	if(photoDifference > photoThreshold && max(leftPhotoValue, rightPhotoValue) < maxSeek){
		return TRUE;
	}
	else if(photoDifference < -photoThreshold && max(leftPhotoValue, rightPhotoValue) < maxSeek){
		return TRUE;
	}else{
		return FALSE;
	}
}

/******************************************************/
int CheckAvoidConditions(){
	int leftIRValue;
	int rightIRValue;
	int avoidThreshold = 500;

	leftIRValue = analog_et(LEFT_IR);
	rightIRValue = analog_et(RIGHT_IR);

	if(leftIRValue > avoidThreshold){
		return TRUE;
	}
	else if(rightIRValue > avoidThreshold){
		return TRUE;
	}else{
		return FALSE;
	}
}

/******************************************************/
int CheckApproachConditions(){
	int leftIRValue;
	int rightIRValue;
	int approachThreshold = 300;

	leftIRValue = analog_et(LEFT_IR);
	rightIRValue = analog_et(RIGHT_IR);

	if(leftIRValue > approachThreshold){
		return TRUE;
	}
	else if(rightIRValue > approachThreshold){
		return TRUE;
	}else{
		return FALSE;
	}
}

/******************************************************/
void EscapeFrontBump(){
	int bumpThreshold = 250;
	int bumpMax = 400;
	int frontBumpValue = analog10(FRONT_BUMP);
	//printf("EscapeFrontBump()\n");
	if(frontBumpValue < bumpThreshold){
		Drive(-0.25, -0.75, 0.30);
		//printf("Drive(-0.25, -0.75, 1.5);\n");
	}
	else if(frontBumpValue >= bumpThreshold && frontBumpValue <= bumpMax){
		Drive(-0.75, -0.25, 0.30);
		//printf("Drive(-0.75, -0.25, 1.5);\n");
	}

}

/******************************************************/
void EscapeBackBump(){
	int bumpThreshold = 250;
	int bumpMax = 400;

	int backBumpValue = analog10(BACK_BUMP);

	if(backBumpValue < bumpThreshold){
		Drive(0.25, 0.75, 0.30);
	}
	else if(backBumpValue >= bumpThreshold && backBumpValue <= bumpMax)
	{
		Drive(0.75, 0.25, 0.30);
	}
}

/******************************************************/
void SeekLight(){
	int leftPhotoValue;
	int rightPhotoValue;
	int photoDifference;
	int photoThreshold = 10;
	int minSeek   = 1023 * 0.45;

	leftPhotoValue = 1023 - analog10(LEFT_PHOTO) + LEFT_PHOTO_OFFSET;
	rightPhotoValue = 1023 - analog10(RIGHT_PHOTO);
	photoDifference = leftPhotoValue - rightPhotoValue;
	//Positive: Turn LEFT, Negative: Turn RIGHT
	printf("lpv: %d, rpv: %d, diff: %d\n", leftPhotoValue, rightPhotoValue, photoDifference);
	if(photoDifference > photoThreshold){// && min(leftPhotoValue, rightPhotoValue) > minSeek){
		Drive(0.30+LEFT_MOTOR_OFFSET, 0.50, 0.25);
	}
	else if(photoDifference < -photoThreshold){// && min(leftPhotoValue, rightPhotoValue) > minSeek){
		Drive(0.50+LEFT_MOTOR_OFFSET, 0.30, 0.25);
	}
}

/******************************************************/
void SeekDark(){
	int leftPhotoValue;
	int rightPhotoValue;
	int photoDifference;
	int photoThreshold = 30;
	int maxSeek   = 1023 * 0.40;

	leftPhotoValue = 1023 - analog10(LEFT_PHOTO);
	rightPhotoValue = 1023 - analog10(RIGHT_PHOTO);
	photoDifference = leftPhotoValue - rightPhotoValue;

	if(photoDifference > photoThreshold && max(leftPhotoValue, rightPhotoValue) < maxSeek){
		Drive(0.60, 0.50, 0.125);
	}
	else if(photoDifference < -photoThreshold && max(leftPhotoValue, rightPhotoValue) < maxSeek){
		Drive(0.50, 0.60, 0.125);
	}

}

/******************************************************/
void Avoid(){
	int leftIRValue;
	int rightIRValue;
	int avoidThreshold = 500;

	leftIRValue = analog_et(LEFT_IR);
	rightIRValue = analog_et(RIGHT_IR);

	if(leftIRValue > avoidThreshold){
		Drive(0.75, 0.25, 0.5);
	}

	else if(rightIRValue > avoidThreshold){
		Drive(0.25, 0.75, 0.5);
	}
}

/******************************************************/
void Approach(){
	int leftIRValue;
	int rightIRValue;
	int approachThreshold = 300;

	leftIRValue = analog_et(LEFT_IR);
	rightIRValue = analog_et(RIGHT_IR);

	if(leftIRValue > approachThreshold){
		Drive(0.25, 0.75, 0.5);
	}

	if(rightIRValue > approachThreshold){
		Drive(0.75, 0.25, 0.5);
	}
}
/******************************************************/
void StraightCruise(){
	Drive(0.60 + LEFT_MOTOR_OFFSET, 0.60, 0.5);
}
/******************************************************/
void StraightBack(){
	int leftPhotoValue;
	int rightPhotoValue;
	int photoDifference;
	leftPhotoValue = 1023 - analog10(LEFT_PHOTO) + LEFT_PHOTO_OFFSET;
	rightPhotoValue = 1023 - analog10(RIGHT_PHOTO);
	photoDifference = leftPhotoValue - rightPhotoValue;
	printf("SeekLight Cond...?\n");
	printf("lpv: %d, rpv: %d, diff: %d\n", leftPhotoValue, rightPhotoValue, photoDifference);
	Drive(-(0.60 + LEFT_MOTOR_OFFSET), -0.60, 0.5);
}
/******************************************************/
void ArcCruise(){
	Drive(0.75, 0.5, 0.5);
}

/******************************************************/
void Drive(float left, float right, float delaySeconds){
	delaySeconds *= 1000;
	motor(LEFT_MOTOR, TOP_SPEED * left);
	motor(RIGHT_MOTOR, TOP_SPEED * right);
	msleep(delaySeconds);
}
/******************************************************/
int min(int first, int second)
{
	int minValue = 0;
	if(first < second){
		minValue = first;
	}else{
		minValue = second;
	}
	return minValue;
}
/******************************************************/
int max(int first, int second)
{
	int maxValue = 0;

	if(first > second){
		maxValue = first;
	}else{
		maxValue = second;
	}

	return maxValue;
}
