#pragma once
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include "MPC.h"

using namespace std;
using namespace Eigen;

extern int nblkdiag(MatrixXd &M, MatrixXd &N, int n);

struct Kinematic_Vehicle
{
	double LFAXIS;
	double LRAXIS;
	double LF;
	double LR;
	double WIDTH;
};

class Planner_mpc : public MPC
{
public:
	Planner_mpc(GRBEnv *planner_env, mutex *planner_mutex);
	~Planner_mpc();

	/* model initialize function */
	void initialize(double Vx, vector<Point> &bound);

	/* load reference path */
	int loadReference(const string fileName);

	/* load obstacle info */
	int loadBound(vector<Point>& bound, vector<Point> &newBound, double r, const string dir);

	/* model step function */
	void step(CAR_STATE * beginState, Path & planned_path);

	/* thread function of model step */
	void threadStep(CAR_STATE * beginState, Path & planned_path);

	/* transform offset description to Point description */
	Point offset2Point(double d_offset, double phi_offset, Point &base_p);

public:

private:
	static constexpr auto Tp = 0.1;
	static constexpr auto NP = 20;
	static constexpr auto NC = 20;
	static constexpr auto NSTATE = 3;
	static constexpr auto NINPUT = 1;
	static constexpr auto NDUMMY = 2;
	static constexpr auto NOUTPUTCON = 8;
	static constexpr auto NETA = 0.3;
	static constexpr auto KAPPA_MAX = 0.02203;
	static constexpr auto REPLANERROR = 0.6;
	static constexpr char ROADFILENAME[] = "straightRoad.csv";
	static constexpr char SAMPLEDATAFILENAME[] = "sampleMatrix.csv";

private:
	static Kinematic_Vehicle Car;
	static P_controller P_Planner;
	static const int rows = NOUTPUTCON * NP;
	static const int cols = NINPUT * NC + NDUMMY;

	double vx;

	MatrixXd Q, Ru, Rdu;
	MatrixXd Phi, Gamma, Ew;
	MatrixXd G_con, E_con;
	VectorXd f, b, Ref;
	
	VectorXd planner_z;
	bool planner_z_isInitialized = false;
	double currentS;

	size_t s_index;

	Path ref_path;
	Path past_path;

	vector<GRBLinExpr> lhs_vec;
	vector<GRBConstr> constraints;
};

