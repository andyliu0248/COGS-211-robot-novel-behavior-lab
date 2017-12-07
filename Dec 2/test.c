#include <math.h>

#define TRUE 1
#define FALSE 0
#define PI 3.14159265358979323846

float vector_length(float vec_x, float vec_y);
float move_to(int x, int y, float curr_angle);
float dot_product(float end_x, float end_y, float init_x, float init_y);
void vector_normalize(float *x, float *y);
int is_parallel_zero(float x1, float y1, float x2, float y2);
void cross_product(float a1, float a2, float a3, float b1, float b2, float b3, float* c1, float* c2, float* c3);
int is_to_the_left(float a1, float a2, float a3, float b1, float b2, float b3);


int main(){
  //printf("%f", vector_length(3, 4));
  //float x1 = 1, y1 = 1;
  //vector_normalize(&x1, &y1);
  //printf("%f, %f\n", x1, y1);
  /*
  float x1=1, y1=1, x2=2, y2=-1;
  printf("%d\n", is_parallel_zero(x1, y1, x2, y2));
  printf("%f, %f, %f, %f\n", x1, y1, x2, y2);
  */
  float a1=1, a2=0, a3=0, b1=0, b2=1, b3=0, c1, c2, c3;
  cross_product(a1, a2, a3, b1, b2, b3, &c1, &c2, &c3);
  printf("(%f, %f, %f), left?: %d\n", c1, c2, c3, is_to_the_left(a1, a2, a3, b1, b2, b3));
  cross_product(b1, b2, b3, a1, a2, a3,&c1, &c2, &c3);
  printf("(%f, %f, %f), left?: %d\n", c1, c2, c3, is_to_the_left(b1, b2, b3, a1, a2, a3));
  float v1=-.53, v2=-.85, v3=0, w1=-147, w2=74, w3=0, r1, r2, r3;
  cross_product(v1, v2, v3, w1, w2, w3, &r1, &r2, &r3);
  printf("(%f, %f, %f), left?: %d\n", r1, r2, r3, is_to_the_left(v1, v2, v3, w1, w2, w3));

}


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

int is_to_the_left_1(x1, y1, x2, y2){
	float epsilon = 0.0001;
	if (fabs(x1) < 0.001){
		printf("Y pointing back, target to the left? %d\n", ((x2 * y1 < 0)? TRUE: FALSE));
		return (x2 * y1 < 0)? TRUE: FALSE;
	}else{
		float slope = y1/x1;
		printf("Y pointing askew, target to the left? %d\n", ((x2 < y2/slope)? TRUE: FALSE));
		return (x2 < y2/slope)? TRUE: FALSE;
	}
}

float dot_product(float end_x, float end_y, float init_x, float init_y){
	return end_x * init_x + end_y * init_y;
}

float vector_length(float vec_x, float vec_y){
	return powf(powf(vec_x, 2.0) + powf(vec_y, 2.0), 0.5);
}

void vector_normalize(float *x, float *y){
	float length = vector_length(*x, *y);
	*x = *x / length;
	*y = *y / length;
}

int is_parallel_zero(float x1, float y1, float x2, float y2){
	vector_normalize(&x1, &y1);
	vector_normalize(&x2, &y2);
	float new_vec_x = x1 + x2;
	float new_vec_y = y1 + y2;
	float epsilon = 0.001;
	float new_length = vector_length(new_vec_x, new_vec_y);
  float sum_length = vector_length(x1, y1) + vector_length(x2, y2);
  printf("new: %f, sum: %f\n, diff: %f\n", new_length, sum_length, fabs(new_length - sum_length));
  return fabs(new_length - sum_length) < epsilon;
}

float move_to(int x, int y, float curr_angle){
	float dir_x = -sin(curr_angle);
	float dir_y = cos(curr_angle);
  printf("angle: %f\n", curr_angle);
  printf("dir x: %f, dir y: %f\n", dir_x, dir_y);
  float dot_p = dot_product(x, y, dir_x, dir_y);
  float len_p = vector_length(dir_x, dir_y)*vector_length(x, y);
  printf("dot: %f, len product: %f\n", dot_p, len_p);
	float cos_angle_contained =  dot_p/ len_p;
  printf("cos: %f\n", cos_angle_contained);
  float angle_contained = acos(cos_angle_contained);
  printf("cos: %f, angle_contained: %f\n", cos_angle_contained, angle_contained);

  float slope = dir_y/dir_x;
	int is_to_the_left = (x < y/slope)? TRUE: FALSE;
	float angle_to_be_turned = (is_to_the_left)? angle_contained : -angle_contained;
  printf("left? %d, angle TBT: %f\n", is_to_the_left, angle_to_be_turned);
  return angle_to_be_turned;
}
