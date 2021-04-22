#pragma once
#include "NetCommIF_CarData.h"
#include <math.h>

constexpr double PI = 3.1415926515;
constexpr double RAD2DEG = 180 / PI;
constexpr double DEG2RAD = PI / 180;
constexpr double Kt = 17;

class Simulator
{
public:
	Simulator(CAR_STATE* state, double xInit, double yInit, double speed, 
		double yawAngle, double yawRate, double t);
	~Simulator();

	void update(float u);

public:
	CAR_STATE* state;
	double Tp;
	const double L = 2.5;
};

