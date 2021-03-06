#pragma once
#include <Windows.h>
#include <vector>
#include <string>
#include <time.h>
#include <conio.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "gurobi_c++.h"
#include "NetCommIF_CarData.h"
#include "Path.h"

using namespace std;
using namespace Eigen;

constexpr auto PI = 3.1415926;
constexpr auto G = 9.8066;
constexpr auto RAD2DEG = 180 / PI;
constexpr auto DEG2RAD = PI / 180;
constexpr auto EPS = 1e-10;

int nblkdiag(MatrixXd &M, MatrixXd &N, int n);

struct P_controller
{
	vector<double> WeightQ;
	vector<double> WeightR;
	vector<double> WeightRdu;
	vector<double> WeightDummy;
	double UMAX;
	double DUMAX;
};

class MPC
{
public:
	GRBEnv *env;
	GRBModel *model;
	GRBVar* vars;
	GRBQuadExpr obj_h;

	vector<Point> leftBound, rightBound;

	bool terminate;
	bool isObsDetected = false;

	MPC() {}
	virtual ~MPC() {}

	double linear_interpolation(double x, vector<double> &xs, vector<double> &ys);

	void linear_interpolation(vector<double> &x, vector<double> &y, vector<double> &xs, vector<double> &ys);
	
};


