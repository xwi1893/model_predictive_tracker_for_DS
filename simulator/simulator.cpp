#include "simulator.h"

P_vehicle Simulator::Car{
	//Mass
	1100,

	//Inertia Z
	2942,

	//lf
	1.0,

	//lr
	1.635,

	//Cf
	6.3713e04,

	//Cr
	7.2995e04,

	//length
	4.4,

	//width
	1.725
};

Simulator::Simulator(const double xInit, const double yInit, const double speed, const double yawAngle, const double yawRate, const double t) :
	Tp(t)
{
	state.dPos[0] = xInit;
	state.dPos[1] = yInit;
	state.fSpeed = speed;
	state.yawAngle = yawAngle;
	state.yawRate = yawRate;
	state.dBeta = 0;
	state.lTotalFrame = 0;
	state.dtime = 0;
}

Simulator::~Simulator()
{
}

void Simulator::update(float u)
{
	state.dPos[0] += state.fSpeed * cos(state.yawAngle) * Tp;
	state.dPos[1] += state.fSpeed * sin(state.yawAngle) * Tp;
	double delta = u * DEG2RAD / Kt;
	state.yawRate = state.fSpeed * tan(delta) / L;
	state.yawAngle += state.yawRate * Tp;
	state.dtime += Tp;
	state.lTotalFrame += 3;
}

void Simulator::dynamic_update(float u)
{
	double vel = state.fSpeed;
	double dBetadt = -2*(Car.CF + Car.CR) * state.dBeta / (Car.MASS*vel) + 2*(Car.CR*Car.LRAXIS - Car.CF*Car.LFAXIS) * state.yawRate / (Car.MASS*vel*vel) - state.yawRate;
	double dGammadt = 2*(Car.CR*Car.LRAXIS - Car.CF*Car.LFAXIS) * state.dBeta / Car.IZ - 2 * (Car.CF*Car.LFAXIS*Car.LFAXIS + Car.CR*Car.LRAXIS*Car.LRAXIS) * state.yawRate/ (Car.IZ*vel);
	state.yawAngle += state.yawRate*Tp;
	state.dBeta += dBetadt*Tp;
	state.yawRate += dGammadt*Tp;
	state.dPos[0] += vel * cos(state.yawAngle) * Tp;
	state.dPos[1] += vel * sin(state.yawAngle) * Tp;
	state.dtime += Tp;
	state.lTotalFrame += 3;
}

void Simulator::addObstacle(const short hModeID, const char cDisp, const float posX, const float posY, const float yaw, const float speed, const float acc)
{
	int i = 0;
	while (i < 10) {
		if (state.stModel[i].hModeID > 0) {
			i++;
		}
		else {
			ST_MODEL_DATA& model = state.stModel[i];
			model.hModeID = hModeID;
			model.cDisp = cDisp;
			model.fPosX = posX;
			model.fPosY = posY;
			model.fIsoYaw = yaw;
			model.fSpeed = speed;
			model.fAccel = acc;
			break;
		}
	}
}
