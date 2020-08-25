#include "Planner_mpc.h"



Kinematic_Vehicle Planner_mpc::Car{
	// len to front axis
	1.0,

	// len to rear axis
	1.635,

	// len to front end
	1.375,

	// len to rear end
	3.025,

	// width
	1.725
};

P_controller Planner_mpc::P_Planner{
	// weight Q, weight epsilon, weight eta
	{2, 20, 1, 10000, 100},
	// weight R,
	{0.1},
	// weight delta u
	{0},
	// umax
	0.7284,
	// delta umax
	-1
};

Planner_mpc::Planner_mpc(GRBEnv *planner_env, mutex *planner_mutex)
{
	env = planner_env;
	model = new GRBModel(*env);
	pMutex = planner_mutex;
	obj_h = 0;
	s_index = 0;
	terminate = false;

	ifstream ref_file(ROADFILENAME);
	vector<Point> ref_waypoints;
	vector<double> ref_s;
	while (ref_file)
	{
		string s;
		if (!getline(ref_file, s)) break;
		istringstream ss(s);
		vector<double> data;
		while (ss)
		{
			string str_data;
			if (!getline(ss, str_data, ',')) break;
			data.push_back(stod(str_data));
		}
		ref_waypoints.emplace_back(data[0], data[1], data[2]);
		ref_s.push_back(data[3]);
	}
	vector<double> ref_kappa(ref_waypoints.size(), 0);
	ref_path = Path(ref_waypoints, ref_kappa, ref_s);
}

Planner_mpc::~Planner_mpc()
{
	delete model;
	model = nullptr;
	if (vars != nullptr)
	{
		delete vars;
		vars = nullptr;
	}
}

void Planner_mpc::initialize(double Vx, vector<Point> &bound)
{
	vx = Vx;

	/* construct discrete matrices(Tp = 0.1) */
	MatrixXd Ad(NSTATE, NSTATE), Bd1(NSTATE, NINPUT), Bd2(NSTATE, NINPUT), Ed(NSTATE, 1);
	Ad << 1, 1.944444, 1.890432,
		0, 1, 1.944444,
		0, 0, 1;
	Bd1 << 0.047261, 0.064815, 0.05;
	Bd2 << 0.015754, 0.032407, 0.05;
	Ed << -1.944444, 0, 0;

	/* state elimination */
	Phi = MatrixXd::Zero(NSTATE * NP, NSTATE);
	Gamma = MatrixXd::Zero(NSTATE * NP, NC);
	Ew = MatrixXd::Zero(NSTATE * NP, NP);
	MatrixXd Phi_part = MatrixXd::Identity(NSTATE, NSTATE);
	MatrixXd Gamma_part = MatrixXd::Zero(NSTATE, NC);
	MatrixXd Ew_part = MatrixXd::Zero(NSTATE, NP);
	for (size_t i = 0; i < NP; i++)
	{
		Phi_part = Ad * Phi_part;
		Phi.block<NSTATE, NSTATE>(NSTATE*i, 0) = Phi_part;

		MatrixXd Ew_tmp = MatrixXd::Zero(NSTATE, NP);
		Ew_tmp.block<NSTATE, 1>(0, i) = Ed;
		Ew_part = Ad * Ew_part + Ew_tmp;
		Ew.block<NSTATE, NP>(NSTATE*i, 0) = Ew_part;

		MatrixXd Gamma_tmp(NSTATE, NC);
		if (i == NP-1)
		{
			Gamma_tmp << MatrixXd::Zero(NSTATE, i), Bd1 + Bd2;			
		}
		else
		{
			Gamma_tmp << MatrixXd::Zero(NSTATE, i), Bd1, Bd2, MatrixXd::Zero(NSTATE, NC - i - 2);
		}

		Gamma_part = Ad * Gamma_part + Gamma_tmp;
		Gamma.block<NSTATE, NC>(NSTATE*i, 0) = Gamma_part;
	}

	MatrixXd PE(NSTATE*NP, NSTATE + NP);
	PE << Phi, Ew;

	/* Determine Q, Ru, Rdu */
	RowVectorXd WeightRu(1), WeightRdu(1), WeightQ(3);
	WeightQ << P_Planner.WeightQ[0], P_Planner.WeightQ[1], P_Planner.WeightQ[2];
	WeightRu << P_Planner.WeightR[0];
	WeightRdu << P_Planner.WeightRdu[0];
	RowVectorXd WQ, WRu, WRdu;
	WQ = WeightQ.replicate(1, NP);
	WRu = WeightRu.replicate(1, NP);
	WRdu = WeightRdu.replicate(1, NP);

	Q = WQ.asDiagonal();
	Ru = WRu.asDiagonal();
	Rdu = WRdu.asDiagonal();

	/* output constraint: kappa */
	MatrixXd G_con_kappa = MatrixXd::Zero(NP, NSTATE*NP);
	nblkdiag((MatrixXd(1, NSTATE) << 0, 0, 1).finished(), G_con_kappa, NP);
	MatrixXd A_kappa = G_con_kappa * Gamma;

	G_con_kappa_rhs = MatrixXd::Zero(NP, NSTATE + NP);
	G_con_kappa_rhs = G_con_kappa * PE;

	zRef = VectorXd::Zero(NSTATE*NP);

	/* construction of matrix H */
	MatrixXd H = MatrixXd::Zero(cols, cols);
	MatrixXd C1 = Matrix<double, NC, NC>::Zero();
	C1.block<NC - 1, NC - 1>(1, 0) = -Matrix<double, NC - 1, NC - 1>::Identity();
	MatrixXd C = Matrix<double, NC, NC>::Identity() + C1;  // difference matrix with 1 input

	H.block<NINPUT*NC, NINPUT*NC>(0,0) = Gamma.transpose()*Q*Gamma + Ru + C.transpose()*Rdu*C;
	H(NINPUT*NC + 1, NINPUT*NC + 1) = P_Planner.WeightQ[4];

	double q[cols][cols], aFix[NP][NC], lb[cols], ub[cols];
	Map<Matrix<double, cols, cols, RowMajor>>(&q[0][0], cols, cols) = H;
	Map<Matrix<double, NP, NC, RowMajor>>(&aFix[0][0], NP, NC) = A_kappa;

	/* construction of input variables and dummy variables */
	fill_n(lb, NC, -P_Planner.UMAX);
	fill_n(ub, NC, P_Planner.UMAX);
	lb[NC] = 0;
	lb[NC + 1] = 0;
	ub[NC] = GRB_INFINITY;
	ub[NC + 1] = 1;
	vars = model->addVars(lb, ub, NULL, NULL, NULL, cols);
	
	int i, j;
	for (i = 0; i < cols; i++)
		for (j = 0; j < cols; j++)
			if (q[i][j] != 0)
				obj_h += q[i][j] * vars[i] * vars[j];        // phi_offset = phi-phi_r, phi_r may be ignored in this.

	/* initialize fixed constraints */
	for (i = 0; i < NP; i++)
	{
		GRBLinExpr lhs = 0;
		for (j = 0; j < NC; j++)
		{
			if (aFix[i][j] != 0) lhs += aFix[i][j] * vars[j];
		}
		GRBConstr constr = model->addConstr(lhs, '<', 0);
		fixConstrs.push_back(constr);
		constr = model->addConstr(lhs, '>', 0);
		fixConstrs.push_back(constr);
	}

	/* save optional environment constraints */
	MatrixXd G_env(NOUTPUTCON, NSTATE);
	G_env << 1, -(Car.LF + Car.LR) / 3.0, 0,
		1, 0, 0,
		1, (Car.LF + Car.LR) / 3.0, 0;
	RowVectorXd g_con_env(NC);
	MatrixXd E_con_tmp = MatrixXd::Zero(NOUTPUTCON*NP, NP);
	if (!nblkdiag((MatrixXd(NOUTPUTCON, 1) << (Car.LF + Car.LR) / 3.0, 0, -(Car.LF + Car.LR) / 3.0).finished(), E_con_tmp, NP)) throw "Size of Ez and E_con not match!";
	MatrixXd E_con(NOUTPUTCON*NP, NSTATE + NP);
	E_con << MatrixXd::Zero(NOUTPUTCON*NP, NSTATE), E_con_tmp;

	G_con_env_rhs = MatrixXd::Zero(3 * NP, NSTATE + NP);
	GRBLinExpr left_dummy = vars[NC] + NETA * vars[NC + 1];
	GRBLinExpr right_dummy = -vars[NC] - NETA * vars[NC + 1];
	for (j = 0; j < 3; j++)
	{
		for (i = 0; i < NP; i++)
		{
			RowVectorXd G_env_row(NSTATE*NP);
			G_env_row << RowVectorXd::Zero(NSTATE*i), G_env.row(j), RowVectorXd::Zero(NSTATE*(NP - i - 1));
			g_con_env = G_env_row * Gamma;
			GRBLinExpr lhs = 0;
			for (int k = 0; k < NC; k++)
			{
				if (g_con_env(k) != 0) lhs += g_con_env(k)*vars[k];
			}

			leftConstrs_lhs.push_back(lhs + left_dummy);
			rightConstrs_lhs.push_back(lhs + right_dummy);
			G_con_env_rhs.row(j*NP + i) = G_env_row * PE + E_con.row(NOUTPUTCON*i + j);
		}
	}

	/* load planning boundary */
	double r_veh = sqrt((Car.LF + Car.LR) * (Car.LF + Car.LR) / 36 + Car.WIDTH * Car.WIDTH / 4);
	loadBound(bound, rightBound, r_veh, "right");
}

//int Planner_mpc::loadReference(const string fileName)
//{
//	ifstream ref_file(fileName);
//	while (ref_file)
//	{
//		string s;
//		if (!getline(ref_file, s)) break;
//		istringstream ss(s);
//		vector<double> data;
//		while (ss)
//		{
//			string str_data;
//			if (!getline(ss, str_data, ',')) break;
//			data.push_back(stod(str_data));
//		}
//		ref_path.emplace_back(data[0], data[1], data[2]);
//		ref_s.push_back(data[3]);
//	}
//	if (!ref_path.empty())
//	{
//		return 0;
//	}
//	else
//	{
//		return -1;
//	}
//}

int Planner_mpc::loadBound(vector<Point>& bound, vector<Point> &newBound, double r, const string dir)
{
	int i;
	double theta;
	int sign;
	if (dir == "right") sign = 1;
	else if (dir == "left") sign = -1;
	for (i = 0; i < bound.size(); i++)
	{
		Vector2d r_vec;
		if (i == 0) {
			Vector2d segment = bound[i + 1] - bound[i];
			Matrix2d R;
			R << 0, -1, 1, 0;
			r_vec = R * segment;

		}
		else if (i >= 1 && i < bound.size() - 1)
		{
			Vector2d v1 = bound[i] - bound[i - 1];
			Vector2d v2 = bound[i + 1] - bound[i];
			theta = acos(v1.dot(v2) / (v1.norm()*v2.norm()));
			theta = 0.5*(PI + theta);
			Matrix2d R;
			R << cos(theta), -sin(theta), sin(theta), cos(theta);
			r_vec = R * v1;
		}
		else if (i == bound.size() - 1)
		{
			Vector2d v = bound[i] - bound[i - 1];
			Matrix2d R;
			R << 0, -1, 1, 0;
			r_vec = R * v;
		}
		r_vec = r_vec / r_vec.norm() *r*sign;
		Point a = bound[i] + r_vec;
		newBound.emplace_back(a);
	}
	return 0;
}


void Planner_mpc::step(CAR_STATE *currentState, Path & planned_path, MPC &tracker, vector<Path> &recorder)
{
	Point currentPos;
	int i, j;
	double d_offset = 0;
	double phi_offset = 0;
	double kappa = 0;

	/* reinitialize planner_z if deviates a lot */
	tracker.beginStateMutex.lock();
	if (tracker.isDeviate_ref) planner_z_isInitialized = false;
	tracker.beginStateMutex.unlock();

	if (!planner_z_isInitialized) currentPos = Point(currentState->dPos[0], currentState->dPos[1], currentState->dAng[2]);
	else {
		tracker.beginStateMutex.lock();
		currentPos = tracker.beginState.first;
		kappa = tracker.beginState.second;
		tracker.beginStateMutex.unlock();
	}

	/* initialize current position and station */
	s_index = 1;
	if (ref_path.size() > 0)
	{
		// d_offset to reference path
		while (s_index <= ref_path.size() - 1)
		{
			Vector2d u = ref_path.waypoints[s_index] - ref_path.waypoints[s_index - 1];
			Vector2d v = currentPos - ref_path.waypoints[s_index];

			double angle = acos(u.dot(v) / (u.norm()*v.norm()+EPS));
			if (angle < PI / 2) s_index++;
			else
			{
				int sgn = (currentPos.orientation(ref_path.waypoints[s_index - 1]) >= ref_path.waypoints[s_index].orientation(ref_path.waypoints[s_index - 1])) ? 1 : -1;
				d_offset = v.norm()*sin(angle)*sgn;
				currentS = ref_path.s[s_index] + v.norm()*cos(angle);
				break;
			}
		}

		// phi_offset to reference path
		double head_angle = currentState->dAng[2];
		phi_offset = head_angle - (currentS - ref_path.s[s_index - 1]) / (ref_path.s[s_index] - ref_path.s[s_index - 1])*ref_path.waypoints[s_index].theta \
			- (ref_path.s[s_index] - currentS) / (ref_path.s[s_index] - ref_path.s[s_index - 1])*ref_path.waypoints[s_index - 1].theta;
	}

	VectorXd planner_z(NSTATE);
	planner_z << d_offset, phi_offset, kappa;
	planner_z_isInitialized = true;

	/* add current planner point to past_path */
	Path future_path;
	future_path.push_back(currentPos, kappa, currentS);

	/* Predict S, x, y in prediction horizon */
	vector<double> predictS, predictS_rear, predictS_front;
	vector<double> predictRef;
	vector<Point> predictPoint, predictPoint_rear, predictPoint_front;
	for (i = 1; i <= NP; i++) {
		predictS_rear.emplace_back(currentS + vx * i*Tp - (Car.LF + Car.LR)/3.0);
		predictS.emplace_back(currentS + vx * i*Tp);
		predictS_front.emplace_back(currentS + vx * i*Tp + (Car.LF + Car.LR)/3.0);
	}
		
	ref_path.linear_interpolation(predictS_rear, predictPoint_rear);
	ref_path.linear_interpolation(predictS, predictPoint);
	ref_path.linear_interpolation(predictS_front, predictPoint_front);

	if (!(refS.empty() || refOffset.empty())) linear_interpolation(predictS, predictRef, refS, refOffset);

	/* calculate h_con, use predicted point to determine left and right bound */
	VectorXd h_con(NOUTPUTCON*NP);


	/* difference matrix, need to change when more than 1 input */
	//MatrixXd C = Matrix<double, NP, NP>::Identity();
	//MatrixXd C1 = Matrix<double, NP, NP>::Zero();
	//C1.block<NP - 1, NP - 1>(1, 0) = -Matrix<double, NP - 1, NP - 1>::Identity();
	//C += C1;

	/* f, b determination */

//	VectorXd w = VectorXd::Ones(cols) * theta_r;            // curved road considered later.
	VectorXd w = VectorXd::Zero(NP);

	for (i = 0; i < NP; i++) zRef(NSTATE*i) = predictRef[i];

	/* set model objective */
	f = RowVectorXd::Zero(cols);
	f.head(NC) = (Phi * planner_z + Ew * w).transpose() * Q * Gamma;  // vector uses head(), tail(), segment(p, q).
	//f.head(NC) = (Phi*planner_z + Ew * w - zRef).transpose()*Q*Gamma;
	f(NC) = P_Planner.WeightQ[3];

	double *c = f.data();
	GRBQuadExpr obj_f = 0;
	for (i = 0; i < cols; i++)
	{
		if (c[i] != 0)	obj_f += c[i] * vars[i];
	}
	model->setObjective(obj_h + obj_f);

	/* set model constraints(rhs) */
	VectorXd zw(NSTATE + NP);
	zw << planner_z, w;
	VectorXd h_fix_rhs = -G_con_kappa_rhs * zw;

	/* revise fixed constraints */
	for (i = 0; i < NP; i++)
	{
		fixConstrs[2 * i].set(GRB_DoubleAttr_RHS, KAPPA_MAX + h_fix_rhs(i));
		fixConstrs[2 * i + 1].set(GRB_DoubleAttr_RHS, -KAPPA_MAX + h_fix_rhs(i));
	}

	/* add environment constraints to model */
	double h_tmp;
	for (j = 0; j < 3; j++)
	{
		vector<Point> *predictPoint_ptr = nullptr;
		switch (j)
		{
		case 0:
			predictPoint_ptr = &predictPoint_rear;
		case 1:
			predictPoint_ptr = &predictPoint;
		case 2:
			predictPoint_ptr = &predictPoint_front;
		}

		for (i = 0; i < predictPoint_ptr->size(); i++)
		{
			Point &point = predictPoint_ptr->at(i);
			if (point.transform2b(h_tmp, rightBound)) {
				GRBConstr constr_env = model->addConstr(rightConstrs_lhs[j*NP + i], '<', h_tmp - NETA - G_con_env_rhs.row(j*NP + i)*zw);
				safeConstrs.push_back(constr_env);
			}
			if (point.transform2b(h_tmp, leftBound)) {
				GRBConstr constr_env = model->addConstr(leftConstrs_lhs[j*NP + i], '>', h_tmp + NETA - G_con_env_rhs.row(j*NP + i)*zw);
				safeConstrs.push_back(constr_env);
			}
		}
	}


	try
	{
		double objVal;
		double sol[cols];
		model->optimize();
		if (model->get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
			objVal = model->get(GRB_DoubleAttr_ObjVal);
			for (int i = 0; i < cols; i++)
				sol[i] = vars[i].get(GRB_DoubleAttr_X);

			VectorXd u(NC);
			for (i = 0; i < NC; i++) u(i) = sol[i];
			VectorXd z_ph = Phi * planner_z + Gamma * u + Ew * w;


			for (int i = 0; i < NP; i++)
			{
				Point future_waypoint = offset2Point(z_ph(NSTATE*i), z_ph(NSTATE*i + 1), predictPoint[i]);
				future_path.push_back(future_waypoint, z_ph(NSTATE*i + 2), currentS + (i + 1) * vx*Tp);
			}

			recorder.push_back(future_path);

			pMutex->lock();
			planned_path = future_path;
			pMutex->unlock();


		}

		for (auto &constr : safeConstrs) model->remove(constr);
		safeConstrs.clear();
		model->update();
	}
	catch (GRBException e)
	{
		cout << "Error code = " << e.getErrorCode() << endl;
		cout << e.getMessage() << endl;
	}
	catch (...)
	{
		cout << "Exception during optimization" << endl;
	}
}


void Planner_mpc::threadStep(CAR_STATE * p_DS_to_C, Path & planned_path, MPC &tracker, vector<Path> &recorder)
{
	long frameCount = 0;
	int key = 0;
	while (!terminate)
	{
		long cFrameCount = p_DS_to_C->lTotalFrame;
		if (cFrameCount>=frameCount + 120)
		{
			frameCount = cFrameCount;
			step(p_DS_to_C, planned_path, tracker, recorder);
			cout << "A new path is planned" << endl;
		}

		if (_kbhit() != 0) key = _getch();
		if (key == 'q') terminate = true;
		if (s_index >= ref_path.size()) terminate = true;
	}

	if (terminate) cout << "The planner is terminated!" << endl;
}

Point Planner_mpc::offset2Point(double d_offset, double phi_offset, Point & base_p)
{
	return Point(base_p.x - d_offset * sin(base_p.theta), base_p.y + d_offset * cos(base_p.theta), base_p.theta + phi_offset);
}
