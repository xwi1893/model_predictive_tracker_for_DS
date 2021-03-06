#pragma once
#include <vector>
#include <string>
#include <math.h>
#include "MPC.h"

using namespace std;
using namespace Eigen;

extern int nblkdiag(MatrixXd &M, MatrixXd &N, int n);
extern vector<float> planner_timeCost;

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
	Planner_mpc(GRBEnv *planner_env);
	~Planner_mpc();

	/* model initialize function */
	void initialize(double Vx, vector<Point> &rBound, vector<Point> &lBound);

	/* load reference path */
	int loadReference(const string fileName);

	/* load obstacle info */
	int loadBound(vector<Point>& bound, vector<Point> &newBound, double r, const string dir);

	/* model step function */
	void step(CAR_STATE * beginState, Path & planned_path, MPC &tracker, vector<Path> & recorder);

	/* thread function of model step */
	void threadStep(CAR_STATE * beginState, Path & planned_path, MPC &tracker, vector<Path> &recorder);

	/* transform offset description to Point description */
	Point offset2Point(double d_offset, double phi_offset, Point &base_p);

public:
	vector<double> refS, refOffset;

private:
	static constexpr auto Tp = 0.1;
	static constexpr auto NP = 30;
	static constexpr auto NC = 30;
	static constexpr auto NSTATE = 3;
	static constexpr auto NINPUT = 1;
	static constexpr auto NDUMMY = 2;
	static constexpr auto NOUTPUTCON = 3;
	static constexpr auto NETA = 0.3;
	static constexpr auto KAPPA_MAX = 0.0318;                 // kappa*vx*vx <= 0.9*mu*g
	static constexpr char ROADFILENAME[] = "csvFile/straightRoad.csv";

private:
	static Kinematic_Vehicle Car;
	static P_controller P_Planner;
	static const int rows = NOUTPUTCON * NP;
	static const int cols = NINPUT * NC + NDUMMY;

	double vx;

	MatrixXd Q, Ru, Rdu;
	MatrixXd Phi, Gamma, Ew;
	MatrixXd G_con_kappa_rhs, G_con_env_rhs;
	VectorXd f, b, zRef;
	

	bool planner_z_isInitialized = false;
	double currentS;

	size_t s_index;

	Path ref_path;

	vector<GRBLinExpr> leftConstrs_lhs, rightConstrs_lhs;
	vector<GRBConstr> safeConstrs;
	vector<GRBConstr> fixConstrs;
};

