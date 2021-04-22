#include "Simulator.h"

Simulator::Simulator(CAR_STATE* state, double xInit, double yInit, double speed, double yawAngle, double yawRate, double t)
{
	this->state = state;
	this->state->dPos[0] = xInit;
	this->state->dPos[1] = yInit;
	this->state->fSpeed = speed;
	this->state->yawAngle = yawAngle;
	this->state->yawRate = yawRate;
	this->state->lTotalFrame = 0;
	this->state->dtime = 0;
	Tp = t;
}

Simulator::~Simulator()
{
}

void Simulator::update(float u)
{
	state->dPos[0] += state->fSpeed * cos(state->yawAngle) * Tp;
	state->dPos[1] += state->fSpeed * sin(state->yawAngle) * Tp;
	double delta = u * DEG2RAD / Kt;
	state->yawRate = state->fSpeed * tan(delta) / L;
	state->yawAngle += state->yawRate * Tp;
	state->dtime += Tp;
	state->lTotalFrame++;
}
