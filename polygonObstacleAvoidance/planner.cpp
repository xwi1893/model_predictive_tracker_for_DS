#include "planner.h"

//using namespace std;

template <typename T> 
inline int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

static MatrixXd dR(double alpha0, double dalpha, double t) {
	MatrixXd M(2, 2);
	M << -dalpha * sin(alpha0 + dalpha * t), -dalpha * cos(alpha0 + dalpha * t),
		dalpha*cos(alpha0 + dalpha * t), -dalpha * sin(alpha0 + dalpha * t);
	return M;
}

static void MultiplyTV(MatrixXd& ret, vector<MatrixXd>& coef, double tV) {
	Eigen::Index r = coef[0].rows();
	Eigen::Index c = coef[0].cols();
	ret = MatrixXd::Zero(r, c);
	if (coef.size() != 4) throw "The size of params to MultiplyTV is wrong!";

	double t = 1;
	for (int i = 0; i < 4; i++) {
		ret = ret + coef[i] * t;
		t = t * tV;
	}
}

static void MultiplyTV(vector<MatrixXd>& ret, vector<MatrixXd>& coef, double tV) {
	Index r = coef[0].rows();
	Index c = coef[0].cols();
	MatrixXd M = MatrixXd::Zero(r, c);
	if (coef.size() != 4) throw "The size of params to MultiplyTV is wrong!";

	double t = 1;
	for (int i = 0; i < 4; i++) {
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

	//double D_array[4] = { -0.046296296296296*vel, -0.083333333333333*vel, 0, 0.1*vel};
	//memcpy(D, D_array, 4 * sizeof(double));
	this->D = (VectorXd(NSTATE) << -0.046296296296296, -0.083333333333333, 0, 0.1).finished();

	A_Tc = (MatrixXd(NSTATE, NSTATE) << 1.0, 0.416666666666667, 0.086805555555556, -0.086805555555556,
		0, 1.0, 0.416666666666667, -0.416666666666667,
		0, 0, 1.0, 0,
		0, 0, 0, 1.0).finished();
	B_Tc = (MatrixXd(NSTATE, NINPUT) << 0.000723379629630, 0.005208333333333, 0.025, 0).finished();
	D_Tc = (MatrixXd(NSTATE, NDIST) << -0.000723379629630, -0.005208333333333, 0, 0.025).finished();

	/* set param.H */
	double H_N[36] = { 0.1, 0, 0, 0, 0, 0,
					   0, 1000, 0, 0, 0, 0,
					   0, 0, 500, 0, 0, 0,
					   0, 0, 0, 1e2, -1e2, 0,
					   0, 0, 0, -1e2, 1e2, 1e8 };
	memcpy(params.H_N, H_N, sizeof(double) * 36);

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


solver_int32_default Planner_mpc::step(CAR_STATE * currentState, Path & planned_path, Environment& env, vector<Planner_solveTime>& timeRecorder)
{
	unique_lock<mutex> plannedPathLock(planned_path.pathMutex);
	while (!planned_path.isNeedReplan) {
		planned_path.replanCv.wait(plannedPathLock);
	}

	clock_t startTime = clock();
	double x = currentState->dPos[0], y = currentState->dPos[1], currentTime = currentState->dtime;

	Waypoint roadLoc = road_ref.cart2frenet(x, y, 0, road_ref.size());
	//currentS = roadLoc.s;
	currentS = roadLoc.s + vel * Tp;
	Waypoint refLoc;
	if (planned_path.size() > 0) {
		interp<double, Waypoint>(planned_path.s, planned_path.waypoints, currentS, refLoc);
	}
	else {
		refLoc = roadLoc;
	}
	plannedPathLock.unlock();

	vector<double> s_predict(NP, 0);      
	for (int i = 0; i < NP; i++) {
		s_predict[i] = currentS + i * vel*Tp;
	}
	interp<double, double>(road_ref.s, road_ref.dkappads, s_predict, dkappads);

	VectorXd eq_c(NSTATE);
	eq_c << -refLoc.offset, -refLoc.theta, -refLoc.kappa, -roadLoc.kappa;

	env.envLock.lock();
	vector<Obstacle*> obstacles = env.dispObstacles;
	env.envLock.unlock();
	
	for (int i = 0; i < NP; i++) {
		if (i > 0) {
			eq_c = -D * vel*dkappads[i-1];
		}
		memcpy(params.c + NSTATE * i, eq_c.data(), sizeof(eq_c));

		double ti = currentTime + (i + 1)*Tp;
		MatrixXd ineq_A = MatrixXd::Zero(32, 6);
		VectorXd ineq_b = VectorXd::Zero(32);

		Eigen::Index ineqStartRow = 0;

		for (int j = 0; j < obstacles.size(); j++) {
			obstacles[j]->motion(ti);
			MatrixXd dR_ij = dR(obstacles[j]->initState.alpha, obstacles[j]->initState.dalpha, ti - obstacles[j]->tStart);
			for (int k = 0; k < 3; k++) {
				vector<Edge> edges;
				createSeparation(s_predict[i] + sOffset[k], dR_ij, *obstacles[j], edges, ti);
				if (!edges.empty()) {
					MatrixXd Gk, bk;
					overApprox(sOffset[k], ti, *obstacles[j], edges, vel*dkappads[i], Gk, bk);  // vel * dkappads[i] !
					ineq_A.block(ineqStartRow, 0, Gk.rows(), 6) = Gk;
					ineq_b.block(ineqStartRow, 0, bk.rows(), 1) = bk;
					ineqStartRow += Gk.rows();
				}
			}
		}

		memcpy(params.A + 192 * i, ineq_A.data(), 192 * sizeof(double));
		memcpy(params.b + 32 * i, ineq_b.data(), 32 * sizeof(double));
	}

	solver_int32_default exit_code = myMPC_FORCESPro_solve(&params, &output, &info, fp);

	double stepT = (clock() - startTime) * 1.0 / CLOCKS_PER_SEC;
	timeRecorder.emplace_back(stepT, info.solvetime);

	return exit_code;
}

void Planner_mpc::threadStep(CAR_STATE * currentState, Path & planned_path, Environment& env, vector<Path>& recorder, vector<Planner_solveTime>& timeRecorder)
{
	while (!term)
	{
		solver_int32_default exit_code = step(currentState, planned_path, env, timeRecorder);
		if (exit_code != OPTIMAL_myMPC_FORCESPro) {
			isOutputOptimal = false;
			cout << "Planner failed to get an optimal result!" << endl;
		}
		else
		{
			isOutputOptimal = true;
			Output2Path(planned_path, recorder);
		}
	}

	cout << "The planner is terminated!" << endl;
}

void Planner_mpc::Output2Path(Path & planned_path, vector<Path>& recorder)
{
	if (!isOutputOptimal) return;
	myMPC_FORCESPro_float* p = (myMPC_FORCESPro_float*)&output;
	double sForward = currentS;
	double sPast = currentS - vel * Tp;
	double u;
	double x[NSTATE];
	VectorXd x_vec(NSTATE);
	planned_path.pathMutex.lock();
	if (planned_path.size() > 0) {
		int i = 0;
		while (i < planned_path.size()) {
			if (planned_path.waypoints[i].s < sPast) {
				planned_path.waypoints.erase(planned_path.waypoints.begin() + i);
				planned_path.dkappads.erase(planned_path.dkappads.begin() + i);
				planned_path.s.erase(planned_path.s.begin() + i);
			}
			else if (planned_path.waypoints[i].s >= currentS) {
				planned_path.waypoints.erase(planned_path.waypoints.begin() + i, planned_path.waypoints.end());
				planned_path.dkappads.erase(planned_path.dkappads.begin() + i, planned_path.dkappads.end());
				planned_path.s.erase(planned_path.s.begin() + i, planned_path.s.end());
				break;
			}
			else i++;
		}
	}
	for (int i = 0; i < 10; i++) {
		u = *p++;
		memcpy(x, p, NSTATE * sizeof(double));
		p += NSTATE+1;
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
			pointForward.kappa = x_vec(2);

			planned_path.push_back(pointForward, 0, sForward);
			sForward += vel * Tc;
			if (j < 3) x_vec = A_Tc * x_vec + B_Tc * u + D_Tc * vel * dkappads[i]; 
		}
	}
	planned_path.isNeedReplan = false;
	recorder.push_back(planned_path);
	planned_path.pathMutex.unlock();
	cout << "A new path is planned" << endl;
}

void Planner_mpc::createSeparation(double s, MatrixXd& dR, Obstacle& obstacle, vector<Edge>& edges, double t)
{
	int sgn_direction = (obstacle.avoidSide == LEFT) ? 1 : -1;
	MatrixXd Vobsr;
	obstacle.curVertices(Vobsr);
	//cout << "Vobsr = " << endl << Vobsr << endl;
	const int n = (int)obstacle.Vertices.cols();

	size_t index = floor((t - obstacle.tStart) / Tp);
	if (index >= obstacle.record.size()) index = obstacle.record.size() - 1;
	ObsRecord& rec = obstacle.record[index];
	MatrixXd TR = rec.TR;
	MatrixXd Txy = rec.Txy;


	MatrixXd V_sh = TR * Vobsr + Txy * MatrixXd::Ones(1, n);
	MatrixXd dV_sh = TR * dR*obstacle.Vertices;

	double* sInit = new double[n];
	double* ds = new double[n];
	double* tKey = new double[n];
	for (int i = 0; i < n; i++) {
		sInit[i] = V_sh(0, i) - s;
		ds[i] = vel - obstacle.currentState.v*cos(obstacle.currentState.theta - rec.p.theta) - dV_sh(0, i);
		tKey[i] = sInit[i] / ds[i];
	}

	//cout << "sInit= " << sInit << endl;
	//cout << "tKey= " << tKey << endl;

	double tLast = 0;
	double tUb = Tp;
	double tKeyMin = 1;
	int tKeyMinIndex;
	vector<edgeNo> firstEdge;
	edgeNo currentEdge;

	for (int k = 0; k < n; k++) {
		int kn = (k + 1) % n;
		if ((sInit[k]<0 && sInit[kn]>0) || (sInit[k]>0 && sInit[kn]<0))
		{
			firstEdge.emplace_back(k, kn);
		}
		double tKey_k = tKey[k];
		if (tKey_k >= tLast && tKey_k < tKeyMin && tKey_k <= Tp) {
			tKeyMin = tKey_k;
			tKeyMinIndex = k;
		}
	}
	if (firstEdge.empty()) {
		if (tKeyMin <= Tp) {
			tLast = tKeyMin;
			firstEdge.emplace_back(tKeyMinIndex, (tKeyMinIndex + 1) % n);
			firstEdge.emplace_back(tKeyMinIndex, (tKeyMinIndex + n - 1) % n);
		}
		else return;
	}


	double y1_mid = V_sh(1, firstEdge[0].first) + V_sh(1, firstEdge[0].second);
	double y2_mid = V_sh(1, firstEdge[1].first) + V_sh(1, firstEdge[1].second);
	if ((y2_mid - y1_mid)*sgn_direction > 0) {
		currentEdge = firstEdge[1];
	}
	else
	{
		currentEdge = firstEdge[0];
	}



	while (tLast < Tp) {
		tUb = Tp;
		double se = sInit[currentEdge.second] - sInit[currentEdge.first];
		double dse = ds[currentEdge.second] - ds[currentEdge.first];
		int k = -1;
		int k_adj = -1;
		if (tKey[currentEdge.first] > tLast && tKey[currentEdge.first] <= tUb) {
			tUb = tKey[currentEdge.first];
			k = currentEdge.first; k_adj = currentEdge.second;
		}
		if (tKey[currentEdge.second] > tLast && tKey[currentEdge.second] <= tUb) {
			tUb = tKey[currentEdge.second];
			k = currentEdge.second; k_adj = currentEdge.first;
		}
		edges.emplace_back(currentEdge, tLast, tUb, -sInit[currentEdge.first], ds[currentEdge.first], se, dse);

		if (k != -1) {
			int knext = (2 * k + n - k_adj) % n;
			if (tKey[knext] > tUb) {
				currentEdge.first = k;
				currentEdge.second = knext;
			}
			else break;
		}
		tLast = tUb;
	}
	delete[] sInit;
	delete[] ds;
	delete[] tKey;
}

void Planner_mpc::overApprox(double s2cg, double tk, Obstacle & obstacle, vector<Edge>& edges, double dist,
	MatrixXd& G_con, MatrixXd& b_con)
{
	MatrixXd G(1, 2);
	G << 1, s2cg;

	int sign = obstacle.avoidSide == LEFT ? -1 : 1;

	double buffer = 0.1;
	size_t n = edges.size();
	double acc = obstacle.initState.acc; 
	double dtheta = obstacle.initState.dtheta; 
	double dalpha = obstacle.initState.dalpha;
	double x, y, v, theta, alpha;

	size_t index = floor((tk - obstacle.tStart) / Tp);
	if (index >= obstacle.record.size()) index = obstacle.record.size() - 1;
	ObsRecord& rec = obstacle.record[index];
	MatrixXd TR = rec.TR;
	MatrixXd Txy = rec.Txy;

	Txy = Txy - TR * (MatrixXd(2, 1) << obstacle.currentState.x, obstacle.currentState.y).finished();
	
	MatrixXd getH(1, 2); getH << 0, 1;
	G_con = MatrixXd::Zero(4 * n, 6);	b_con = MatrixXd::Zero(4 * n, 1);

	for (size_t i = 0; i < n; i++)
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
			G_con(4 * i + k, 5) = -1;
			b_con.block<1, 1>(4 * i + k, 0) = -G * GAMMA_k.block<2, 1>(2, 0)*dist;
		}
		//cout << "G_con = " << endl << G_con << endl;
		//cout << "b_con = " << endl << b_con << endl;

		obstacle.motion(tk + t1);
		v = obstacle.currentState.v;
		theta = obstacle.currentState.theta;
		x = obstacle.currentState.x; y = obstacle.currentState.y;
		alpha = obstacle.currentState.alpha;

		vector<MatrixXd> coef_xyc, coef_R, coef_lambdaR;
		Vector2d V1 = obstacle.Vertices.col(edge.edge_no.first);
		Vector2d V2 = obstacle.Vertices.col(edge.edge_no.second);
		//cout << "V1 = " << V1 << endl;
		//cout << "V2 = " << V2 << endl;
		if (abs(dtheta) < 1e-3) {
			Taylor::Cxy(x, y, v, acc, theta, coef_xyc);
		}
		else
		{
			Taylor::Cxy_dtheta(x, y, v, acc, theta, dtheta, coef_xyc);
		}
		Taylor::CR(alpha, dalpha, coef_R);
		Taylor::ClambdaR(alpha, dalpha, edge.ds, edge.dsEdge, 
			edge.s + edge.ds*t1, edge.sEdge + edge.dsEdge*t1, coef_lambdaR);

		vector<MatrixXd> xyc_ap, R_ap, lambdaR_ap;
		MultiplyTV(xyc_ap, coef_xyc, t_);
		MultiplyTV(R_ap, coef_R, t_);
		MultiplyTV(lambdaR_ap, coef_lambdaR, t_);
		for (int k = 0; k < 4; k++) {
			VectorXd sh = xyc_ap[k] + R_ap[k] * V1;
//			Vector2d hcon = TR * (xyc_ap[k] + R_ap[k] * V1 + lambdaR_ap[k] * (V2 - V1)) + Txy;
//			b_con[4 * i + k] += hcon(1)*sign - buffer;
//			cout << "sh = " << endl << sh << endl;
//			cout << "tmp = " << endl << std::setprecision(9) << getH*TR*( sh + lambdaR_ap[k] * (V2 - V1) )<< endl;
			b_con.block<1, 1>(4 * i + k, 0) = sign * (b_con.block<1, 1>(4 * i + k, 0) + getH * TR*(sh + lambdaR_ap[k] * (V2 - V1)) + getH * Txy) - buffer*MatrixXd::Ones(1,1);
		}
	}
//	cout << "b_con = " << endl << b_con << endl;
}
