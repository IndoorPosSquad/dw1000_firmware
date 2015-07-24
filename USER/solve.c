#include "solve.h"
#include "math.h"
#include "utils.h"
#include "USART.h"

extern int debug_lvl;

#define DIM 3
#define S(n) ((n) * (n))
#define LEN3D(a, b, c, d, e, f) sqrt(S(a - d) + S(b - e) + S(c - f))


float sgn(float x) {
	return x >= 0 ? 1.0 : -1.0;
}

float Det(float arcs[DIM][DIM],int n) {
	int i, j, k;

	float ans = 0;
	float tmp[DIM][DIM] = {0};
	float t;

	if(n == 2)
		return arcs[0][0] * arcs[1][1] -
			arcs[1][0] * arcs[0][1];

	for (i = 0; i < n; i++) {
		for (j = 0; j < n-1; j++)
			for (k = 0; k < n-1; k++)
				tmp[j][k] =
					arcs[j + 1][(k >= i) ? k+1 : k];

		t = Det(tmp, n-1);
		if (i % 2 == 0)
			ans += arcs[0][i] * t;
		else
			ans -= arcs[0][i] * t;
	}
	if (ans == 0) {
		ans = 0.01;
	}
	return ans;
}


xyz solve_3d(xyz pl_xyz[], float pranges[]) {
	int i, j;
	xyz result = {0, 0, 0};
	xyz txyz[3];
	float h_BA, h_CA;
	float _AB, _AC, _BC, _cos_BAC;
	float _trans[3][3] = {0};
	float _d, _H;
	float D[3] = {0};
	float AB, AC, BC, cos_BAC;
	float d, H;
	float trans[3][3] = {0};
	float tmp[3][3] = {0};
	float temp;
	float h, d1, d2, d3, d4, COS, X, Y, err;
	float _P, _S, _n, _q, _p, _r, _m, _l;
	float _eular_det[3][3];
	float _V2, _V;

	float CD, AD, sub;

	float res[3];
	float target_xyz[3];
	float ans[3];
	// 概述
	// 1. 求原坐标系中三个卫星构成的平面，
	//    旋转成平面(即纵坐标相同的平面)的变换矩阵，
	//    (原坐标系指客观存在的那个坐标系，
	//    变换之后的坐标系就是变换坐标系)
	//    这里分了两步变换
	//
	// 2. 由测的三个距+三卫星两两之间的距离，解
	//    由三卫星加未知点构成的四面体，得在变换
	//    坐标系中未知点的位置
	//
	// 3. 由1中求得的变换关系将2中求得的解变换到
	//    原坐标系中



	// xyz为直角坐标内的一个坐标

	// 三个伪卫星构成的三角形, 以伪卫星0为高度基准
	// 将三个卫星投影到一个与xOy平面平行的面上,
	// 在这个投影面上的三卫星放在txyz中
	// 以下设三星分别为A B C

	for (i = 0; i < 3; i++) {
		txyz[i].x = pl_xyz[i].x;
		txyz[i].y = pl_xyz[i].y;
		txyz[i].z = pl_xyz[i].z;
	}

	// 以卫星0的高度为基准，将1/2卫星与0卫星的高度差记在h_2, h_3中
	h_BA = txyz[1].z - txyz[0].z;
	h_CA = txyz[2].z - txyz[0].z;
	txyz[1].z = pl_xyz[0].z;
	txyz[2].z = pl_xyz[0].z;

	DEBUG3(("txyz:\n"));
	for (i = 0; i < 3; i++) {
		DEBUG3(("%d\n", i));
		DEBUG3(("x %f, y %f, z %f\n",
			txyz[i].x, txyz[i].y, txyz[i].z));
		DEBUG3(("\n"));
	}

	// 计算投影面上的三边长与夹角
	_AB = sqrt(pow(txyz[1].x - txyz[0].x, 2) +
		   pow(txyz[1].y - txyz[0].y, 2) +
		   pow(txyz[1].z - txyz[0].z, 2));
	_AC = sqrt(pow(txyz[2].x - txyz[0].x, 2) +
		   pow(txyz[2].y - txyz[0].y, 2) +
		   pow(txyz[2].z - txyz[0].z, 2));
	_BC = sqrt(pow(txyz[2].x - txyz[1].x, 2) +
		   pow(txyz[2].y - txyz[1].y, 2) +
		   pow(txyz[2].z - txyz[1].z, 2));
	_cos_BAC = (pow(_AC, 2) + pow(_AB, 2) - pow(_BC, 2)) /
		(2 * _AC * _AB);

	DEBUG3(("_AB: %f\n", _AB));
	DEBUG3(("_AC: %f\n", _AC));
	DEBUG3(("_BC: %f\n", _BC));
	DEBUG3(("_cos_BAC: %f\n", _cos_BAC));

	// 计算变换矩阵1
	// X
	_trans[0][0] = (txyz[1].x - txyz[0].x) / _AB;
	_trans[0][1] = (txyz[1].y - txyz[0].y) / _AB;
	_trans[0][2] = (txyz[1].z - txyz[0].z) / _AB;

	DEBUG3(("_trans:\n"));
	for (i = 0; i < 3; i++) {
	       for (j = 0; j < 3; j++) {
		       DEBUG3(("%f ", _trans[i][j]));
	       }
	       DEBUG3(("\n"));
	}

	_d = _AC * _cos_BAC;
	_H = sqrt(pow(_AC, 2) - pow(_d, 2));

	DEBUG3(("_d %f\n", _d));
	DEBUG3(("_H %f\n", _H));

	D[0] = txyz[0].x + _d * _trans[0][0];
	D[1] = txyz[0].y + _d * _trans[0][1];
	D[2] = txyz[0].z + _d * _trans[0][2];
	//Y
	_trans[1][0] = (txyz[2].x - D[0]) / _H;
	_trans[1][1] = (txyz[2].y - D[1]) / _H;
	_trans[1][2] = (txyz[2].z - D[2]) / _H;

	_trans[2][0] = 0;
	_trans[2][1] = 0;
	_trans[2][2] = 1;

	DEBUG3(("_trans:\n"));
	for (i = 0; i < 3; i++) {
	       for (j = 0; j < 3; j++) {
		       DEBUG3(("%f ", _trans[i][j]));
	       }
	       DEBUG3(("\n"));
	}

	// 求原坐标系中三卫星构成的三角的边长与夹角
	AB = sqrt(pow(pl_xyz[1].x - pl_xyz[0].x, 2) +
		  pow(pl_xyz[1].y - pl_xyz[0].y, 2) +
		  pow(pl_xyz[1].z - pl_xyz[0].z, 2));
	AC = sqrt(pow(pl_xyz[2].x - pl_xyz[0].x, 2) +
		  pow(pl_xyz[2].y - pl_xyz[0].y, 2) +
		  pow(pl_xyz[2].z - pl_xyz[0].z, 2));
	BC = sqrt(pow(pl_xyz[2].x - pl_xyz[1].x, 2) +
		  pow(pl_xyz[2].y - pl_xyz[1].y, 2) +
		  pow(pl_xyz[2].z - pl_xyz[1].z, 2));
	cos_BAC = (pow(AC, 2) + pow(AB, 2) - pow(BC, 2)) /
		(2 * AC * AB);

	DEBUG3(("AB: %f\n", AB));
	DEBUG3(("AC: %f\n", AC));
	DEBUG3(("BC: %f\n", BC));
	DEBUG3(("cos_BAC: %f\n", cos_BAC));

	d = AC * cos_BAC;
	H = sqrt(pow(AC, 2) - pow(d, 2));

	DEBUG3(("d: %f\n", d));
	DEBUG3(("H: %f\n", H));

	// 计算变换矩阵2
	// X
	trans[0][0] = _AB / AB;
	trans[0][1] = 0;
	trans[0][2] = h_BA / AB;

	trans[1][0] = (_d - (_AB * d / AB)) / H;
	trans[1][1] = _H / H;
	trans[1][2] = (h_CA - (h_BA * d / AB)) / H;

	DEBUG3(("trans:\n"));
	for (i = 0; i < 3; i++) {
	       for (j = 0; j < 3; j++) {
		       DEBUG3(("%f ", trans[i][j]));
	       }
	       DEBUG3(("\n"));
	}

	// 求余子式
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			tmp[i][j] = trans[i][j];

	trans[2][0] = tmp[0][1] * tmp[1][2] - tmp[1][1] * tmp[0][2];
	trans[2][1] = -(tmp[0][0] * tmp[1][2] - tmp[0][2] * tmp[1][0]);
	trans[2][2] = tmp[0][0] * tmp[1][1] - tmp[0][1] * tmp[1][0];

	temp = sqrt(pow(trans[2][0], 2) +
		    pow(trans[2][1], 2) +
		    pow(trans[2][2], 2));

	trans[2][0] /= temp;
	trans[2][1] /= temp;
	trans[2][2] /= temp;
	// 求余子式结束

	DEBUG3(("trans:\n"));
	for (i = 0; i < 3; i++) {
	       for (j = 0; j < 3; j++) {
		       DEBUG3(("%f ", trans[i][j]));
	       }
	       DEBUG3(("\n"));
	}

	h = 0.0;
	d1 = 0.0; d2 = 0.0; d3 = 0.0; d4 = 0.0;
	COS = 0.0;
	X = 0.0; Y = 0.0;
	err = 0.0;

	// 海仑三角公式求底面积
	_P = (BC + AB + AC) / 2;
	_S = sqrt(_P * (_P - BC) * (_P - AC) * (_P - AB));

	DEBUG3(("_P: %f\n", _P));

	// 欧拉四面体公式求体积
	_n = BC;
	_q = AC;
	_p = AB;
	_r = pranges[0];
	_m = pranges[1];
	_l = pranges[2];

	/* float _eular_det[3][3] = */
	/*	{{S(_p), (S(_p)+S(_q)-S(_n))/2, (S(_p)+S(_r)-S(_m))/2}, */
	/*	 {(S(_p)+S(_q)-S(_n))/2, S(_q), (S(_q)+S(_r)-S(_l))/2}, */
	/*	 {(S(_p)+S(_r)-S(_m))/2, (S(_q)+S(_r)-S(_l))/2, S(_r)}}; */
	_eular_det[0][0] = S(_p);
	_eular_det[0][1] = (S(_p)+S(_q)-S(_n))/2;
	_eular_det[0][2] = (S(_p)+S(_r)-S(_m))/2;
	_eular_det[1][0] = (S(_p)+S(_q)-S(_n))/2;
	_eular_det[1][1] = S(_q);
	_eular_det[1][2] = (S(_q)+S(_r)-S(_l))/2;
	_eular_det[2][0] = (S(_p)+S(_r)-S(_m))/2;
	_eular_det[2][1] = (S(_q)+S(_r)-S(_l))/2;
	_eular_det[2][2] = S(_r);


	DEBUG3(("_eular_det:\n"));
	for (i = 0; i < 3; i++) {
	       for (j = 0; j < 3; j++) {
		       DEBUG3(("%f ", _eular_det[i][j]));
	       }
	       DEBUG3(("\n"));
	}


	_V2 = Det(_eular_det, 3);
	_V = sqrt(_V2/36.0);

	DEBUG3(("_V2: %f\n", _V2));
	DEBUG3(("_V: %f\n", _V));

	// 求未知点到三卫星平面的高度
	h = _V / _S * 3.0;

	// 由高度得未知点的横纵坐标
	d1 = sqrt(pow(pranges[0], 2) - pow(h, 2));
	d2 = sqrt(pow(pranges[1], 2) - pow(h, 2));
	d3 = sqrt(pow(pranges[2], 2) - pow(h, 2));
	COS = (d1 * d1 + AB * AB - d2 * d2) / (2 * d1 * AB);
	X = d1 * COS;
	Y = sqrt(pow(d1, 2) - pow(X, 2));

	CD = (_S * 2 / AB);
	AD = sqrt(S(AC) - S(CD));

	sub = LEN3D(AD, CD, 0, X, Y, 0);
	if (fabs(sub - d3) > 0.03) {
		Y = - Y;
	}

	// 未知点在变换空间的坐标
	res[0] = X;
	res[1] = Y;

	#ifndef GROUND_ANCHOR
	// 放在下方
	res[2] = -h;
	#elif
	res[2] = h;
	#endif

	// 分两步将结果变换回原坐标系
	// 矩阵乘 target_xyz = res * trans
	for (i = 0; i < 3; i++) {
		target_xyz[i] = 0.0;
		for (j = 0; j < 3; j++)
			target_xyz[i] += res[j] * trans[j][i];
	}

	DEBUG3(("target_xyz:\n"));
	for (i = 0; i < 3; i++) {
		DEBUG3(("%f \n", target_xyz[i]));
	}
	DEBUG3(("\n"));

	// 矩阵乘 ans = target_xyz * _trans
	for (i = 0; i < 3; i++) {
		ans[i] = 0.0;
		for (j = 0; j < 3; j++)
			ans[i] += target_xyz[j] * _trans[j][i];
	}

	DEBUG3(("ans:\n"));
	for (i = 0; i < 3; i++) {
		DEBUG3(("%f \n", ans[i]));
	}
	DEBUG3(("\n"));

	// 变换坐标系是以0卫星的坐标作为原点
	// 因此将卫星0的坐标加上去
	result.x = ans[0] + pl_xyz[0].x;
	result.y = ans[1] + pl_xyz[0].y;
	result.z = ans[2] + pl_xyz[0].z;

	return result;
}

void solve_2d(float reciever[2][2], float pseudolites[2][2], float pranges1, float pranges2) {
	float origin[2];
	float len;
	float tan_theta;
	float sin_theta;
	float cos_theta;
	float d1;
	float h1;
	float invrotation[2][2];
	float pranges[2];

	DEBUG3(("\nPseudolites\n%f %f %f %f\n", pseudolites[0][0], pseudolites[0][1], pseudolites[1][0], pseudolites[1][1]));
	DEBUG3(("\npr1 %f pr2 %f\n", pranges1, pranges2));

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
