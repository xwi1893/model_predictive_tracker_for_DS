#pragma once
#include <vector>
#include <math.h>
#include <Windows.h>
#include <conio.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include "MpcTracker.h"
#include "interp.hpp"
#include "NetCommIF_CarData.h"
#include "Waypoint.h"
#include "Path.h"

using namespace std;
using namespace Eigen;

constexpr float Kt = 17;
constexpr double EPS = 1e-6;
constexpr double PI = 3.1415926;
constexpr auto DEG2RAD = PI / 180;
constexpr auto RAD2DEG = 180 / PI;

class Tracker_mpc
{
public:
	Tracker_mpc();
	~Tracker_mpc();

	/* model initialize function */
	void initialize();

	/* model step function */
	solver_int32_default step(CAR_STATE *currentState, Path& planned_path);

	/* get output */
	double *getOutput();

	/* thread step function */
	void threadStep(CAR_STATE * p_DS_to_C, float * p_C_to_DS, Path &planned_path);

public:
	Path ref_path;
	Path road_ref;

	bool terminate = false;

private:
	static constexpr auto Tc = 0.025;
	static constexpr auto NP = 30;
	static constexpr auto NSTATE = 4;
	static constexpr auto NINPUT = 1;

private:
	double u_next[1];
	double vx;

	MatrixXd A, D, mat_H;

	MpcTracker_params params;
	MpcTracker_info info;
	MpcTracker_output output;
	FILE* fp;
};

