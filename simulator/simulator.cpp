#include "simulator.h"

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
	state.lTotalFrame++;
}

void Simulator::dynamic_update(float u)
{
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
