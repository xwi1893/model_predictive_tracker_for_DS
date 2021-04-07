#pragma once
#include <Windows.h>
#include <vector>
#include <unordered_set>
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
#include "tracker.h"
#include "Obstacle.h"
#include "myMPC_FORCESPro.h"
#include "NetCommIF_CarData.h"
#include "Waypoint.h"
#include "Path.h"
#include "Taylor.h"

using namespace std;
using namespace Eigen;

typedef pair<int, int> edgeNo;
struct Edge {
	double t1, t2;
	double s, ds, sEdge, dsEdge;
	edgeNo edge_no;

	Edge(edgeNo& no, double t1, double t2, double s, double ds, double sEdge, double dsEdge) :
		edge_no(no), t1(t1), t2(t2), s(s), ds(ds), sEdge(sEdge), dsEdge(dsEdge) {}
};

class Planner_mpc
{
public:
	Planner_mpc();
	~Planner_mpc();

	/* model initialize function */
	void initialize(double vCar);

	/* model step function */
	solver_int32_default step(CAR_STATE * currentState, Path & planned_path, Environment& env);

	/* thread function of model step */
	void threadStep(CAR_STATE * beginState, Path & planned_path, Environment& env);

	/* transform output to path*/
	void Output2Path(Path & planned_path);

	/* create separation */
	vector<Edge> createSeparation(double s, MatrixXd& dR, Obstacle& obstacle, MatrixXd& T);

	/* over approximation of constraints */
	void overApprox(double s2cg, double tk, Obstacle& obstacle, vector<Edge>& edges, double dist, MatrixXd& T, MatrixXd& G_con, MatrixXd& b_con);

public:
	Path road_ref, ref_path;

private:
	static constexpr auto Tp = 0.1;
	static constexpr auto Tc = 0.025;
	static constexpr auto NP = 40;
	static constexpr auto NSTATE = 4;
	static constexpr auto NINPUT = 1;
	static constexpr auto NDIST = 1;
	static constexpr auto BUFFER = 0.1;
	static constexpr char ROADFILENAME[] = "config/Road.csv";

private:
	static const int rows;
	static const int cols;

	double vel;
	double sOffset[3];

	bool terminate = false;
	bool isOutputOptimal = true;
	double currentS;
	vector<double> dkappads;

	myMPC_FORCESPro_params params;
	myMPC_FORCESPro_info info;
	myMPC_FORCESPro_output output;
	FILE* fp;

	MatrixXd A_Tc, B_Tc, D_Tc;
	vector<MatrixXd> GAMMA;
	VectorXd D;
};
