#pragma once

#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <fstream>
#include <iostream>
#include "gurobi_c++.h"
#include "NetCommIF_CarData.h"

using namespace std;
using namespace Eigen;

#define TS 0.025
#define NP 30
#define UMAX 0.35
#define DUMAX 0.02
#define Kt 17
#define EPS 1e-6
#define PI 3.1415926
#define NSTATE 4
constexpr auto DEG2RAD = PI / 180;
constexpr auto RAD2DEG = 180 / PI;

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
};

struct P_controller
{
	double WeightQ[NSTATE];
	double WeightRu;
	double WeightRdu;
};

struct Point
{
	double x;
	double y;
public:
	Point(double x, double y) : x(x), y(y) {}
	Vector2d operator- (Point &p)
	{
		double x_ = x - p.x;
		double y_ = y - p.y;
		Vector2d u(x_, y_);
		return u;
	}
};


/* Class declaration for model model_1_UDP */
class MpcController {
	/* public data and function members */
public:
	/* No. of variables */
	static const int rows = 2 * NP;
	static const int cols = NP;

	/* External inputs */

	/* External outputs */
	double sol[cols];
	double u_next[1];

	/* Embedded variable*/
	GRBEnv *env;
	double kappa;
	double Vx;

	MatrixXd Q, Ru, Rdu;
	MatrixXd H, A;
	VectorXd f, b;
	MatrixXd Phi, Gamma, Ew;

	/* model initialize function */
	void initialize();
	int loadReference(const string fileName);

	/* model step function */
	void updateModel(CAR_STATE *currentState, size_t &s_index);
	void step();

	/* model terminate function */
	void terminate();

	/* Constructor */
	MpcController();

	/* Destructor */
	~MpcController();

	/* Get output */
	double* getOutput();

	/* private data and function members */
private:
	/* Tunable parameters */
	static P_vehicle CClassHatchback;
	static P_controller P_MPC;
	vector<Point> ref_path;

	bool dense_optimization(double* c, double* q, double* a, char*   sense, double* rhs, double* lb, double* ub, char* vtype, double* solution, double* objvalP);
};