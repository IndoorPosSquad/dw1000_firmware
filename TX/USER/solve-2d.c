#include "stm32f10x.h"
#include "math.h"

float origin[2];
float len;
float tan_theta;
float sin_theta;
float cos_theta;
float d1;
float h1;
float invrotation[2][2];
float pranges[2];

float sgn(float x) { return x >= 0 ? 1.0 : -1.0; }

void solve_2d(float reciever[2][2], float pseudolites[2][2], float pranges1, float pranges2) {
	printf("\nPseudolites\n%f %f %f %f\n", pseudolites[0][0], pseudolites[0][1], pseudolites[1][0], pseudolites[1][1]);
	printf("\npr1 %f pr2 %f\n", pranges1, pranges2);

	pranges[0] = pranges1;
	pranges[1] = pranges2;

	origin[0] = pseudolites[0][0];
	origin[1] = pseudolites[0][1];

	pseudolites[0][0] = 0;
	pseudolites[0][1] = 0;
	pseudolites[1][0] = pseudolites[1][0] - origin[0];
	pseudolites[1][1] = pseudolites[1][1] - origin[1];

	len = sqrt(pow(pseudolites[1][0], 2) + pow(pseudolites[1][1], 2));

	tan_theta = pseudolites[1][1] / pseudolites[1][0];
	cos_theta = sgn(pseudolites[1][0]) / sqrt(pow(tan_theta, 2) + 1);
	sin_theta = sgn(pseudolites[1][1]) * fabs(tan_theta) / sqrt(pow(tan_theta, 2) + 1);

	invrotation[0][0] = cos_theta;
	invrotation[0][1] = -sin_theta;
	invrotation[1][0] = sin_theta;
	invrotation[1][1] = cos_theta;

	d1 = ((pow(pranges[0], 2) - pow(pranges[1], 2)) / len + len) / 2;

	h1 = sqrt(pow(pranges[0], 2) - pow(d1, 2));

	reciever[0][0] = d1;
	reciever[0][1] = h1;
	reciever[1][0] = d1;
	reciever[1][1] = -h1;

	reciever[0][0] = invrotation[0][0] * d1 + invrotation[0][1] * h1;
	reciever[0][1] = invrotation[1][0] * d1 + invrotation[1][1] * h1;
	reciever[0][0] += origin[0];
	reciever[0][1] += origin[1];

	reciever[1][0] = invrotation[0][0] * d1 + invrotation[0][1] * -h1;
	reciever[1][1] = invrotation[1][0] * d1 + invrotation[1][1] * -h1;
	reciever[1][0] += origin[0];
	reciever[1][1] += origin[1];
}
