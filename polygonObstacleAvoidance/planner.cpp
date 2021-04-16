#include "planner.h"

//using namespace std;

static MatrixXd dR(double alpha0, double dalpha, double t) {
	MatrixXd M(2, 2);
	M << -dalpha * sin(alpha0 + dalpha * t), -dalpha * cos(alpha0 + dalpha * t),
		dalpha*cos(alpha0 + dalpha * t), -dalpha * sin(alpha0 + dalpha * t);
	return M;
}

static void MultiplyTV(MatrixXd& ret, vector<MatrixXd>& coef, double tV) {
	static const int r = coef[0].rows();
	static const int c = coef[0].cols();
	ret = MatrixXd::Zero(r, c);
	if (coef.size() != 4) throw "The size of params to MultiplyTV is wrong!";

	double t = 1;
	for (int i = 0; i < 4; i++) {
		ret = ret + coef[i] * t;
		t = t * tV;
	}
}

static void MultiplyTV(vector<MatrixXd>& ret, vector<MatrixXd>& coef, double tV) {
	const int r = coef[0].rows();
	const int c = coef[0].cols();
	MatrixXd M = MatrixXd::Zero(r, c);
	if (coef.size() != 4) throw "The size of params to MultiplyTV is wrong!";
	//DenseBase<double>::Scalar t = 1;
	double t = 1;
	for (int i = 0; i < 4; i++) {
//		cout << coef[i] * t << endl;
		M = M + coef[i] * t;
		t *= tV;
		ret.push_back(M);
	}
}

Planner_mpc::Planner_mpc()
{
	// load path
	if (!road_ref.loadFile("../config/road.csv")) throw "Failed to load reference path!";

	// initialize
	double vCar = 16.66667;
	initialize(vCar);
}

Planner_mpc::~Planner_mpc()
{
}

void Planner_mpc::initialize(double vCar)
{
	vel = vCar;
	sOffset[0] = -2.2916666667; sOffset[1] = -0.825; sOffset[2] = 0.6416666667;

	this->D = (VectorXd(NSTATE) << -0.046296296296296, -0.083333333333333, 0, 0.1).finished();

	A_Tc = (MatrixXd(NSTATE, NSTATE) << 1.0, 0.416666666666667, 0.086805555555556, -0.086805555555556,
		0, 1.0, 0.416666666666667, -0.416666666666667,
		0, 0, 1.0, 0,
		0, 0, 0, 1.0).finished();
	B_Tc = (MatrixXd(NSTATE, NINPUT) << 0.000723379629630, 0.005208333333333, 0.025, 0).finished();
	D_Tc = (MatrixXd(NSTATE, NDIST) << -0.000723379629630, -0.005208333333333, 0, 0.025).finished();


	/* read GAMMA */
	MatrixXd gamma(NSTATE + NINPUT + NDIST, NSTATE + NINPUT + NDIST);
	ifstream file;
	file.open("../config/planner_config.txt", ios::in);
	if (!file.good()) cout << "Failed to read GAMMA!" << endl;


	int i = 0;
	while (file)
	{
		string s;
		if (!getline(file, s)) break;
		istringstream ss(s);
		if (i < NSTATE + NINPUT + NDIST) {
			for (size_t j = 0; j < NSTATE + NINPUT + NDIST; j++) {
				string str_data;
				if (!getline(ss, str_data, ',')) break;
				gamma(i, j) = stod(str_data);
			}
		}
		if (++i == NSTATE + NINPUT + NDIST) {
			GAMMA.push_back(gamma);
			i = 0;
		}
	}
}


solver_int32_default Planner_mpc::step(CAR_STATE * currentState, Path & planned_path, Environment& env)
{
	unique_lock<mutex> plannedPathLock(planned_path.pathMutex);
	while (!planned_path.isNeedReplan) {
		planned_path.replanCv.wait(plannedPathLock);
	}

	double x = currentState->dPos[0], y = currentState->dPos[1];

	Waypoint roadLoc = road_ref.cart2frenet(x, y, 0, road_ref.size());
	currentS = roadLoc.s;
	Waypoint refLoc;
	if (planned_path.size() > 0) {
		interp<double, Waypoint>(planned_path.s, planned_path.waypoints, currentS, refLoc);
	}
	else {
		refLoc = roadLoc;
	}
	plannedPathLock.unlock();

	vector<double> s_predict;
	for (int i = 0; i < NP; i++) {
		s_predict.push_back(currentS + i * vel*Tp);
	}
	interp<double, double>(road_ref.s, road_ref.dkappads, s_predict, dkappads);

	VectorXd eq_c(NSTATE);
	eq_c << -refLoc.offset, -refLoc.theta, -refLoc.kappa, -roadLoc.kappa;

	env.envLock.lock();
	vector<Obstacle> obstacles = env.obstacles;
	env.envLock.unlock();
	
	for (int i = 0; i < NP; i++) {
		if (i > 0) {
			eq_c = -D * vel*dkappads[i-1];
		}
		memcpy(params.c + NSTATE * i, eq_c.data(), sizeof(eq_c));

		double ti = currentState->dtime + i * Tp;
		MatrixXd ineq_A = MatrixXd::Zero(32, 5);
		VectorXd ineq_b = VectorXd::Zero(32);

		int ineqStartRow = 0;

		for (int j = 0; j < obstacles.size(); j++) {
			obstacles[j].motion(ti);
			MatrixXd dR_ij = dR(obstacles[j].initState.alpha, obstacles[j].initState.dalpha, ti - obstacles[j].tStart);
			for (int k = 0; k < 3; k++) {
				MatrixXd T;
				vector<Edge> edges = createSeparation(s_predict[i] + sOffset[k], dR_ij, obstacles[j], T);
				if (!edges.empty()) {
					MatrixXd Gk, bk;
					overApprox(sOffset[k], ti, obstacles[j], edges, dkappads[i], T, Gk, bk);
					ineq_A.block(ineqStartRow, 0, Gk.rows(), 5) = Gk;
					ineq_b.block(ineqStartRow, 0, bk.rows(), 1) = bk;
					ineqStartRow += Gk.rows();
				}
			}
		}

		memcpy(params.A + 160 * i, ineq_A.data(), 160*sizeof(double));
		memcpy(params.b + 32 * i, ineq_b.data(), 32 * sizeof(double));
	}

	solver_int32_default exit_code = myMPC_FORCESPro_solve(&params, &output, &info, fp);

	return exit_code;
}

void Planner_mpc::threadStep(CAR_STATE * currentState, Path & planned_path, Environment& env)
{
	while (!terminate)
	{
		solver_int32_default exit_code = step(currentState, planned_path, env);
		if (exit_code != OPTIMAL_myMPC_FORCESPro) {
			isOutputOptimal = false;
			cout << "Planner failed to get an optimal result!" << endl;
		}
		else
		{
			isOutputOptimal = true;
			Output2Path(planned_path);
		}
	}

	cout << "The planner is terminated!" << endl;
}

void Planner_mpc::Output2Path(Path & planned_path)
{
	if (!isOutputOptimal) return;
	myMPC_FORCESPro_float* p = (myMPC_FORCESPro_float*)&output;
	double sForward = currentS;
	double u;
	double x[NSTATE];
	VectorXd x_vec(NSTATE);
	planned_path.pathMutex.lock();
	if (planned_path.size() > 0) planned_path.clear();
	for (int i = 0; i < 10; i++) {
		u = *p++;
		memcpy(x, p, NSTATE * sizeof(double));
		p += NSTATE;
		x_vec = Map<VectorXd>(x, NSTATE);
		for (int j = 0; j < 4; j++) {
			double d = x_vec(0);
			double e_phi = x_vec(1);
			Waypoint pointForward;
			interp<double, Waypoint>(road_ref.s, road_ref.waypoints, sForward, pointForward);
			pointForward.x = pointForward.x - d * sin(pointForward.theta);
			pointForward.y = pointForward.y + d * cos(pointForward.theta);
			pointForward.offset = d;
			pointForward.theta = e_phi;

			planned_path.push_back(pointForward, 0, sForward);
			sForward += vel * Tc;
			if (j < 3) x_vec = A_Tc * x_vec + B_Tc * u + D_Tc * dkappads[i]; 
		}
	}
	planned_path.isNeedReplan = false;
	planned_path.pathMutex.unlock();
	cout << "A new path is planned" << endl;
}

vector<Edge> Planner_mpc::createSeparation(double s, MatrixXd& dR, Obstacle& obstacle, MatrixXd& T)
{
	int sgn_direction = (obstacle.avoidSide == LEFT) ? 1 : -1;
	MatrixXd V, Vobsr;
	obstacle.curVertices(V, Vobsr);
	double dalpha = obstacle.initState.dalpha;
	int n = obstacle.Vertices.cols();

	double px = obstacle.currentState.x; double py = obstacle.currentState.y;

	Waypoint obstacle_sh = road_ref.cart2frenet(px, py, 0, road_ref.size());
	double theta_sh = obstacle_sh.theta;
	MatrixXd R(2, 2);
	R << cos(theta_sh), -sin(theta_sh), sin(theta_sh), cos(theta_sh);
	MatrixXd Txy = (MatrixXd(2, 1) << obstacle_sh.s, obstacle_sh.offset).finished() - R * (MatrixXd(2, 1) << px, py).finished();
	T = (MatrixXd(3, 3) << cos(theta_sh), -sin(theta_sh), Txy(0,0),
		sin(theta_sh), cos(theta_sh), Txy(1,0),
		0, 0, 1).finished();

	ArrayXXd sInit, ds;
	MatrixXd V_sh(V.rows() + 1, V.cols());
	V_sh << V, MatrixXd::Ones(1, V.cols());
	V_sh = T * V_sh;
	sInit = V_sh.row(0).array() - s * ArrayXXd::Ones(1, n);
	//cout << "s = " << s << endl;
	//cout << "V= " << V_sh.row(0).array() << endl;
	//cout << "sInit= " << sInit << endl;
	ds = (vel - obstacle.initState.vx) * ArrayXXd::Ones(1, n) - (dR * obstacle.Vertices).row(0).array();
	ArrayXXd tKey = sInit / ds;
	//cout << "tKey= " << tKey << endl;

	double tLast = 0;
	double tUb = Tp;
	double tKeyMin = 1;
	int tKeyMinIndex;
	vector<edgeNo> firstEdge;
	edgeNo currentEdge;
	vector<Edge> edges;

	for (int k = 0; k < n; k++) {
		int kn = (k + 1) % n;
		if (sInit(k)*sInit(kn) < 0) {
			firstEdge.emplace_back(k, kn);
		}
		if (tKey(k) >= tLast && tKey(k) < tKeyMin && tKey(k) <= Tp) {
			tKeyMin = tKey(k);
			tKeyMinIndex = k;
		}

	}
	if (firstEdge.empty() && tKeyMin<=Tp) {
		firstEdge.emplace_back( tKeyMinIndex, (tKeyMinIndex + 1) % n );
		firstEdge.emplace_back( tKeyMinIndex, (tKeyMinIndex + n - 1) % n );
	}
	else return edges;


	double y1_mid = Vobsr(1, firstEdge[0].first) + Vobsr(1, firstEdge[0].second);
	double y2_mid = Vobsr(1, firstEdge[1].first) + Vobsr(1, firstEdge[1].second);
	if ((y2_mid - y1_mid)*sgn_direction > 0) {
		currentEdge = firstEdge[1];
	}
	else
	{
		currentEdge = firstEdge[0];
	}



	while (tLast < Tp) {
		tUb = Tp;
		double se = sInit(currentEdge.second) - sInit(currentEdge.first);
		double dse = ds(currentEdge.second) - ds(currentEdge.first);
		int k = -1;
		int k_adj = -1;
		if (tKey(currentEdge.first) > tLast && tKey(currentEdge.first) <= tUb) {
			tUb = tKey(currentEdge.first);
			k = currentEdge.first; k_adj = currentEdge.second;
		}
		if (tKey(currentEdge.second) > tLast && tKey(currentEdge.second) <= tUb) {
			tUb = tKey(currentEdge.second);
			k = currentEdge.second; k_adj = currentEdge.first;
		}
		edges.emplace_back(currentEdge, tLast, tUb, -sInit(currentEdge.first), ds(currentEdge.first), se, dse);

		if (k != -1) {
			currentEdge.first = k;
			currentEdge.second = (2 * k + n - k_adj) % n;
		}
		tLast = tUb;
	}

	return edges;
}

void Planner_mpc::overApprox(double s2cg, double tk, Obstacle & obstacle, vector<Edge>& edges, double dist, MatrixXd & T, 
	MatrixXd& G_con, MatrixXd& b_con)
{
	//MatrixXd G(1, 4);
	//G << 1, s2cg, 0, 0;
	MatrixXd G(1, 2);
	G << 1, s2cg;


	int sign = obstacle.avoidSide == LEFT ? -1 : 1;

	double buffer = 0.1;
	int n = edges.size();
	double acc = obstacle.initState.acc; 
	double dtheta = obstacle.initState.dtheta; 
	double dalpha = obstacle.initState.dalpha;
	double x, y, v, theta, alpha;

	MatrixXd getH(1, 2); getH << 0, 1;
	MatrixXd TR = T.block<2, 2>(0, 0);
	MatrixXd Txy = T.block<2, 1>(0, 2);

	G_con = MatrixXd::Zero(4 * n, 5);
	b_con = MatrixXd::Zero(4 * n, 1);

	for (int i = 0; i < n; i++)
	{
		Edge& edge = edges[i];
		double t1 = edge.t1;
		double t_ = edge.t2 - edge.t1;

		MatrixXd Gamma_tmp;
		MultiplyTV(Gamma_tmp, GAMMA, t1);
		vector<MatrixXd> Gamma_vec;
		MultiplyTV(Gamma_vec, GAMMA, t_);
		for (int k = 0; k < 4; k++) {
			MatrixXd GAMMA_k = Gamma_vec[k]*Gamma_tmp;
			G_con.block<1, 5>(4 * i + k, 0) = sign * G * GAMMA_k.block<2, 5>(2, 1);
			b_con.block<1, 1>(4 * i + k, 0) = -G * GAMMA_k.block<2, 1>(2, 0)*dist;
		}

		obstacle.motion(tk + t1);
		v = obstacle.currentState.v;
		theta = obstacle.currentState.theta;
		x = obstacle.currentState.x; y = obstacle.currentState.y;
		alpha = obstacle.currentState.alpha;

		vector<MatrixXd> coef_xyc, coef_R, coef_lambdaR;
		Vector2d V1 = obstacle.Vertices.col(edge.edge_no.first);
		Vector2d V2 = obstacle.Vertices.col(edge.edge_no.second);
		
		Taylor::Cxy(x, y, v, acc, theta, coef_xyc);
		Taylor::CR(alpha, dalpha, coef_R);
		Taylor::ClambdaR(alpha, dalpha, edge.ds, edge.dsEdge, 
			edge.s + edge.ds*t1, edge.sEdge + edge.dsEdge*t1, coef_lambdaR);

		vector<MatrixXd> xyc_ap, R_ap, lambdaR_ap;
		MultiplyTV(xyc_ap, coef_xyc, t_);
		MultiplyTV(R_ap, coef_R, t_);
		MultiplyTV(lambdaR_ap, coef_lambdaR, t_);
		for (int k = 0; k < 4; k++) {
			VectorXd sh = xyc_ap[k] + R_ap[k] * V1;
			b_con.block<1, 1>(4 * i + k, 0) = sign * (b_con.block<1, 1>(4 * i + k, 0) + getH * TR*(sh + lambdaR_ap[k] * (V2 - V1)) + getH * Txy) + buffer*MatrixXd::Ones(1,1);
		}
	}
}
