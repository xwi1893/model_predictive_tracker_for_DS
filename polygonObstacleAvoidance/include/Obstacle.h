#pragma once
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <mutex>
#include <condition_variable>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <Windows.h>
#include "Path.h"
#include "NetCommIF_CarData.h"

using namespace std;

typedef Eigen::Matrix<double, 2, Dynamic> MatrixVerd;

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

struct ObsRecord
{
	double time;
	Waypoint p;
	MatrixXd TR;
	MatrixXd Txy;

	ObsRecord(const double t, Waypoint& waypoint, MatrixXd& tr, MatrixXd& txy): time(t), p(waypoint), TR(tr), Txy(txy) {}
};

class Obstacle {
public:
	short ID;
	State initState;
	State currentState;
	double tStart, tEnd;
	MatrixXd Vertices;
	Side avoidSide = LEFT;
	vector<ObsRecord> record;

public:
	Obstacle() {}
	Obstacle(short id, double x, double y, double v, double a, double theta, double dtheta,
		double alpha, double dalpha, double tStart, double tEnd, MatrixXd& Vobs, Side side) :
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

	void curVertices(Eigen::MatrixXd& V_rel) {
		double alpha = currentState.alpha;
		Eigen::Matrix2d R;
		R << cos(alpha), -sin(alpha), sin(alpha), cos(alpha);
		V_rel = R * Vertices;
	}

	~Obstacle() {}
};

class Environment {
public:
	vector<Obstacle> obstacles;
	vector<Obstacle*> dispObstacles;
	mutex envLock;
	bool hasObstaclesRead = false;

	Environment() {}
	~Environment() {}

	void updateObstacles(const CAR_STATE* state, Path& planned_path) {
		assert(hasObstaclesRead == true);
		bool isNeedReplan = false;
		for (int i = 0; i < 10; i++) {
			const ST_MODEL_DATA& model = state->stModel[i];
			if (model.hModeID == -1) continue;
			bool isInObstacles = false;
			for (auto it = dispObstacles.begin(); it != dispObstacles.end(); it++) {
				if ((*it)->ID == model.hModeID) {
					isInObstacles = true;
					if (!model.cDisp) dispObstacles.erase(it);
					break;
				}
			}

			if (!isInObstacles && model.cDisp) {
				for (Obstacle& obs : obstacles) {
					if (model.hModeID == obs.ID) {
						obs.tStart = state->dtime;
						obs.tEnd = (model.fAccel > -0.01) ? state->dtime + 10 : state->dtime - model.fSpeed / model.fAccel;
						dispObstacles.push_back(&obs);
						break;
					}
				}
				isNeedReplan = true;
			}
		}
		if (isNeedReplan) {
			planned_path.isNeedReplan = true;
			planned_path.replanCv.notify_all();
		}
	}

	Obstacle readObstacleFromIni(const char* fileName, LPCSTR section) {
		const int SIZE = 512;
		char receiveStr[SIZE];

		GetPrivateProfileStringA(section, "id", "0", receiveStr, SIZE, fileName);
		short id = (short)strtol(receiveStr, NULL, 0);

		vector<LPCSTR> keyNames = { "x", "y", "v", "a", "theta", "dtheta", "alpha", "dalpha", "tStart", "tEnd" };
		vector<double> obstacleInfo;
		for (int i = 0; i < keyNames.size(); i++) {
			GetPrivateProfileStringA(section, keyNames[i], "0", receiveStr, SIZE, fileName);
			obstacleInfo.push_back(strtod(receiveStr, NULL));
		}

		double x, y, v, a, theta, dtheta, alpha, dalpha, tStart, tEnd;
		x = obstacleInfo[0]; y = obstacleInfo[1]; v = obstacleInfo[2]; a = obstacleInfo[3]; theta = obstacleInfo[4];
		dtheta = obstacleInfo[5]; alpha = obstacleInfo[6]; dalpha = obstacleInfo[7]; tStart = obstacleInfo[8]; tEnd = obstacleInfo[9];

		GetPrivateProfileStringA(section, "avoidSide", "default", receiveStr, SIZE, fileName);
		Side avoidSide = (strcmp(receiveStr, "left") == 0) ? LEFT : RIGHT;


		GetPrivateProfileStringA(section, "vertices", "default", receiveStr, SIZE, fileName);
		string line(receiveStr);
		istringstream ss(line);
		vector<double> vertices_vec;
		while (ss) {
			string str_data;
			if (!getline(ss, str_data, ',')) break;
			vertices_vec.push_back(stod(str_data));
		}
		const int verticesCols = vertices_vec.size() / 2;
		Map<Matrix<double, 2, Dynamic, RowMajor>> vertices2Map(vertices_vec.data(), 2, verticesCols);
		MatrixXd Vertices(vertices2Map);

		Obstacle obs(id, x, y, v, a, theta, dtheta, alpha, dalpha, tStart, tEnd, Vertices, avoidSide);
		return obs;
	}

	void readIni(vector<LPCSTR>& keyNames, const char* fileName, Path& road_ref) {
		const double Tp = 0.1;
		for (int i = 0; i < keyNames.size(); i++) {
			Obstacle obs = readObstacleFromIni(fileName, keyNames[i]);
			double tEnd = obs.initState.acc > -0.01 ? 10 : -obs.initState.v / obs.initState.acc;
			obs.tEnd = tEnd;
			double tStart = 0;
			while (tStart <= tEnd) {
				obs.motion(tStart);
				Waypoint obs_sh = road_ref.cart2frenet(obs.currentState.x, obs.currentState.y, 0, road_ref.size());
				double theta_sh = obs_sh.theta;
				MatrixXd TR = (MatrixXd(2, 2) << cos(theta_sh), -sin(theta_sh), sin(theta_sh), cos(theta_sh)).finished();
				MatrixXd Txy = (MatrixXd(2, 1) << obs_sh.s, obs_sh.offset).finished();
				obs.record.emplace_back(tStart, obs_sh, TR, Txy);
				tStart += Tp;
			}
			obstacles.push_back(obs);
		}
		hasObstaclesRead = true;
	}

	void threadStep(CAR_STATE* state, Path& planned_path) {
		long frameCount = 0;
		while (!term) {
			long cFrameCount = state->lTotalFrame;
			if (cFrameCount > frameCount) {
				frameCount = cFrameCount;
				envLock.lock();
				updateObstacles(state, planned_path);
				envLock.unlock();
			}
		}
	}
};