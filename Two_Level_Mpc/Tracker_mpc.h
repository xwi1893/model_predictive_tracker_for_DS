#pragma once
#include "MPC.h"

extern int nblkdiag(MatrixXd &M, MatrixXd &N, int n);
extern vector<float> tracker_timeCost;
extern vector<float> tracker_Output;
extern vector<bool> getOptimal;

struct P_vehicle
{
	double MASS;
	double IZ;
	double LFAXIS;
	double LRAXIS;
	double CF;
	double CR;
	double LENGTH;
	double WIDTH;
	double KT;
	double ARMAX;
	double GAMMAMAX;
};


class Tracker_mpc : public MPC
{
public:
	Tracker_mpc(GRBEnv *tracker_env);
	~Tracker_mpc();

	/* model initialize function */
	void initialize();

	/* receive from planner */
	int receiveFromPlanner(MPC &planner);

	/* model step function */
	void updateModel(CAR_STATE *currentState, Path &planned_path);
	void step();

	/* thread function of model step */
	void threadStep(CAR_STATE *p_DS_to_C, float *p_C_to_DS, Path &planned_path);

	/* get output */
	double *getOutput();

public:
	Path ref_path;


private:
	static constexpr auto Tc = 0.025;
	static constexpr auto NP = 30;
	static constexpr auto NC = 30;
	static constexpr auto NSTATE = 4;
	static constexpr auto NINPUT = 1;
	static constexpr auto NDUMMY = 2;
	static constexpr char FrontTyreLookup[] = "csvFile/frontTyreLookup.csv";

private:
	static P_vehicle Car;
	static P_controller P_TRACKER;
	static const int rows = 4 * NP;
	static const int cols = NINPUT * NC + NDUMMY;
	static const int num_fyf = 256;

	double sol[cols];
	double fyf[num_fyf], slipA[num_fyf];

	MatrixXd Q, Ru, Rdu;
	MatrixXd Phi, Gamma, Ew;
	VectorXd f, b;

	MatrixXd G_con_sta_rhs;
	MatrixXd G_con_env_rhs;

	size_t s_index;
	int trackSteps;

	double u_next[1];
	double vx, currentS;

	vector<GRBLinExpr> gsta_lhs_vec;
	vector<GRBLinExpr> genv_lhs_vec;
	vector<GRBConstr> sta_con;
	vector<GRBConstr> env_con;
};

