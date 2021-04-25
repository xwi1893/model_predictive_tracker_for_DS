#pragma once
#include "NetCommIF_CarData.h"
#include <math.h>

constexpr double PI = 3.1415926515;
constexpr double RAD2DEG = 180 / PI;
constexpr double DEG2RAD = PI / 180;
constexpr double Kt = 17;

struct P_vehicle
{
	double MASS;
	double IZ;
	double LFAXIS;
	double LRAXIS;
	double CF;
	double CR;
	double LENGTH;
	double WIDTH;
};

class Simulator
{
public:
	Simulator(const double xInit, const double yInit, const double speed,
		const double yawAngle, const double yawRate, const double t);
	~Simulator();

	void update(float u);
	
	void dynamic_update(float u);

	void addObstacle(const short hModeID, const char cDisp, const float posX, const float posY, 
		const float yaw, const float speed, const float acc);

public:
	CAR_STATE state;
	double Tp;
	const double L = 2.5;
	
private:
	static P_vehicle Car;
};
