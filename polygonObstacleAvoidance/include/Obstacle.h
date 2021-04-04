#pragma once
#include <vector>
#include <mutex>
#include <condition_variable>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "NetCommIF_CarData.h"

using namespace std;


struct State {
	double x; 
	double y;
	double v, vx, vy;
	double acc;
	double theta, dtheta;
	double alpha, dalpha;
	double t;
};

enum Side
{
	LEFT,
	RIGHT
};

class Obstacle {
public:
	short ID;
	State initState;
	State currentState;
	double tStart, tEnd;
	Eigen::MatrixXd Vertices;
	Side avoidSide = LEFT;

public:
	Obstacle() {}
	Obstacle(short id, double x, double y, double v, double a, double theta, double dtheta,
		double alpha, double dalpha, double tStart, double tEnd, Eigen::MatrixXd& Vobs, Side side) :
		ID(id), tStart(tStart), tEnd(tEnd), Vertices(Vobs), avoidSide(side) {
		initState = { x, y, v, v*cos(theta), v*sin(theta), a, theta, dtheta, alpha, dalpha, 0 };
		currentState = initState;
	}

	void set_avoidSide(Side side) {
		this->avoidSide = side;
	}

	void motion(double t) {
		if (initState.t != 0) throw "The obstacle is not initialized!";

		double x = initState.x; 
		double y = initState.y;
		double v = initState.v;
		double acc = initState.acc;
		double alpha = initState.alpha; double dalpha = initState.dalpha;
		double theta = initState.theta; double dtheta = initState.dtheta;
		currentState.t = t;
		t = (t >= tEnd) ? tEnd - tStart : t - tStart;

		currentState.v = v + acc * t;
		currentState.alpha = alpha + dalpha * t;
		currentState.theta = theta + dtheta * t;
		currentState.vx = currentState.v*cos(currentState.theta);
		currentState.vy = currentState.v*sin(currentState.theta);
		
		if (dtheta == 0) {
			currentState.x = x + v * cos(theta)*t + 0.5*acc*cos(theta)*t*t;
			currentState.y = y + v * sin(theta)*t + 0.5*acc*sin(theta)*t*t;
		}
		else {
			currentState.x = x + v * (sin(theta + dtheta * t) - sin(theta)) / dtheta + acc * (cos(theta + dtheta * t) - cos(theta) + dtheta * t*sin(theta + dtheta * t)) / (dtheta*dtheta);
			currentState.y = y + v * (cos(theta) - cos(theta + dtheta * t)) / dtheta + acc * (sin(theta + dtheta * t) - sin(theta) - dtheta * t*cos(theta + dtheta * t)) / (dtheta*dtheta);
		}			
	}

	void curVertices(Eigen::MatrixXd& V, Eigen::MatrixXd& V_rel) {
		double alpha = currentState.alpha;
		Eigen::Matrix2d R;
		R << cos(alpha), -sin(alpha), sin(alpha), cos(alpha);
		size_t n = Vertices.cols();
		V_rel = R * Vertices;
		V = V_rel + (Eigen::MatrixXd(2, 1) << currentState.x, currentState.y).finished().replicate(1, n);
	}

	~Obstacle() {}
};

class Environment {
public:
	vector<Obstacle> obstacles;
	mutex envLock;

	Environment() {}
	~Environment() {}

	void updateObstacles(CAR_STATE* state, Path& planned_path) {
		bool isNeedReplan = false;
		for (int i = 0; i < 5; i++) {
			ST_MODEL_DATA& model = state->stModel[i];
			bool isInObstacles = false;
			for (auto it = obstacles.begin(); it != obstacles.end(); it++) {
				if (it->ID == model.hModeID) {
					isInObstacles = true;
					if (!model.cDisp) obstacles.erase(it);
					break;
				}
			}

			if (!isInObstacles && model.cDisp) {
				MatrixXd Vobs(2, 4);
				Vobs << -1, 1, 1, -1, -1, -1, 1, 1;
				Obstacle obs(model.hModeID, model.fPosX, model.fPosY, model.fSpeed, model.fAccel, model.fIsoYaw,
					0, 0, 0, state->dtime, state->dtime - model.fSpeed / model.fAccel, Vobs, RIGHT);
				obstacles.push_back(obs);
				isNeedReplan = true;
			}
		}
		if (isNeedReplan) {
			planned_path.isNeedReplan = true;
			planned_path.replanCv.notify_all();
		}
	}

	void threadStep(CAR_STATE* state, condition_variable& replanSig) {
		long frameCount = 0;
		int trackSteps = 0;
		bool terminate = false;
		while (!terminate) {
			long cFrameCount = state->lTotalFrame;
			if (cFrameCount >= frameCount + 1) {
				frameCount = cFrameCount;
				envLock.lock();
				updateObstacles(state, replanSig);
				envLock.unlock();
			}
		}
	}
};