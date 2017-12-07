// File: 1443 Nov 17 copy.c
#include <math.h>
#include <time.h>
#include <stdlib.h>

/* --------------------------------------------------------------------------*/
/*                           Function Declarations                           */
/* --------------------------------------------------------------------------*/

// Non-Behavioral Primitives
float random_float(float lower, float upper);



/* --------------------------------------------------------------------------*/
/*                            Function Definitions                           */
/* --------------------------------------------------------------------------*/

int main()
{
	srand(time(NULL));


	int i = 0;
	while (i < 10){
		printf("%f\n", random_float(-1.0, 1.0));
		i++;
	}
	return 0;
}

float random_float(float lower, float upper)
{
	float interval = upper - lower;
	if (interval < 0){
		interval = -interval;
	}
	float r0_1 = (float)rand()/(float)RAND_MAX;
	return lower + r0_1 * interval;
}
