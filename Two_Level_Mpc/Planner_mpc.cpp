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
	{0.1, 10, 1, 10000, 100},
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


	/* output constraint */
	MatrixXd Gz(NOUTPUTCON, NSTATE);
	G_con = MatrixXd::Zero(NOUTPUTCON*NP, NSTATE*NP);
	Gz << 1, -(Car.LF + Car.LR)/3.0, 0,
		1, 0, 0,
		1, (Car.LF + Car.LR)/3.0, 0,
		0, 0, 1,
		-1, (Car.LF + Car.LR)/3.0, 0,
		-1, 0, 0,
		-1, -(Car.LF + Car.LR)/3.0, 0,
		0, 0, -1;
	if (!nblkdiag(Gz, G_con, NP)) throw "Size of Gz and G_con not match!";

	MatrixXd Ez(NOUTPUTCON, 1);
	Ez << (Car.LF + Car.LR)/3.0, 0, -(Car.LF + Car.LR)/3.0, 0, -(Car.LF + Car.LR)/3.0, 0, (Car.LF + Car.LR)/3.0, 0;
	E_con = MatrixXd::Zero(NOUTPUTCON*NP, NP);
	if (!nblkdiag(Ez, E_con, NP)) throw "Size of Ez and E_con not match!";

	MatrixXd G_dummy_p(4, 2), G_dummy;
	G_dummy_p << -1, -NETA, -1, -NETA, -1, -NETA, 0, 0;
	G_dummy = G_dummy_p.replicate(2 * NP, 1);

	MatrixXd A(NOUTPUTCON*NP, NINPUT*NP + NDUMMY);
	A << G_con * Gamma, G_dummy;                      // assign value to A


	double q[cols][cols], a[rows][cols], lb[cols], ub[cols];

	/* construction of matrix H */
	MatrixXd H = MatrixXd::Zero(cols, cols);
	MatrixXd C1 = Matrix<double, NC, NC>::Zero();
	C1.block<NC - 1, NC - 1>(1, 0) = -Matrix<double, NC - 1, NC - 1>::Identity();
	MatrixXd C = Matrix<double, NC, NC>::Identity() + C1;  // difference matrix with 1 input

	H.block<NINPUT*NC, NINPUT*NC>(0,0) = Gamma.transpose()*Q*Gamma + Ru + C.transpose()*Rdu*C;
	H(NINPUT*NC + 1, NINPUT*NC + 1) = P_Planner.WeightQ[4];

	Map<Matrix<double, cols, cols, RowMajor>>(&q[0][0], cols, cols) = H;
	Map<Matrix<double, rows, cols, RowMajor>>(&a[0][0], rows, cols) = A;

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

	for (i = 0; i < rows; i++)
	{
		GRBLinExpr lhs = 0;
		for (j = 0; j < cols; j++)
		{
			if (a[i][j] != 0) lhs += a[i][j] * vars[j];
		}
		lhs_vec.emplace_back(lhs);
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


void Planner_mpc::step(CAR_STATE *currentState, Path & planned_path)
{
	Point currentPos(currentState->dPos[0], currentState->dPos[1], currentState->dAng[2]);
	int i;

	/* reinitialize planner_z if deviates a lot */
	if (planned_path.size() > 0)
	{
		Point &plannedPos = planned_path.waypoints[0];
		if (currentPos.distance(plannedPos) > REPLANERROR) planner_z_isInitialized = false;
	}

	/* initialize current position and station */
	if (!planner_z_isInitialized)
	{
		if (past_path.size() > 0) past_path.clear();
		s_index = 1;
		double d_offset = 0;
		double phi_offset = 0;
		double kappa = 0;
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
		planner_z = VectorXd::Zero(NSTATE);
		planner_z << d_offset, phi_offset, kappa;
		planner_z_isInitialized = true;
	}

	/* add current planner point to past_path */
	Point currentSPoint = ref_path.linear_interpolation(currentS, s_index);
	Point currentPlannerPoint = offset2Point(planner_z(0), planner_z(1), currentSPoint);
	past_path.push_back(currentPlannerPoint, planner_z(2), currentS);
	if (past_path.size() > 3) past_path.pop_first();

	/* Predict S, x, y in prediction horizon */
	vector<double> predictS, predictS_rear, predictS_front;
	vector<Point> predictPoint, predictPoint_rear, predictPoint_front;
	for (i = 1; i <= NP; i++) {
		predictS_rear.emplace_back(currentS + vx * i*Tp - (Car.LF + Car.LR)/3.0);
		predictS.emplace_back(currentS + vx * i*Tp);
		predictS_front.emplace_back(currentS + vx * i*Tp + (Car.LF + Car.LR)/3.0);
	}
		
	ref_path.linear_interpolation(predictS_rear, predictPoint_rear, s_index);
	ref_path.linear_interpolation(predictS, predictPoint, s_index);
	ref_path.linear_interpolation(predictS_front, predictPoint_front, s_index);


	/* calculate h_con, use predicted point to determine left and right bound */
	VectorXd h_con(NOUTPUTCON*NP);
	for (i = 0; i < NP; i++)
	{
		double tmp_b = 0;
		predictPoint_rear[i].transform2b(tmp_b, rightBound);
		h_con(NOUTPUTCON*i) = tmp_b;
		predictPoint[i].transform2b(tmp_b, rightBound);
		h_con(NOUTPUTCON*i + 1) = tmp_b;
		predictPoint_front[i].transform2b(tmp_b, rightBound);
		h_con(NOUTPUTCON*i + 2) = tmp_b;
		h_con(NOUTPUTCON*i + 3) = KAPPA_MAX;
		predictPoint_rear[i].transform2b(tmp_b, leftBound);
		h_con(NOUTPUTCON*i + 4) = -tmp_b;
		predictPoint[i].transform2b(tmp_b, leftBound);
		h_con(NOUTPUTCON*i + 5) = -tmp_b;
		predictPoint_front[i].transform2b(tmp_b, leftBound);
		h_con(NOUTPUTCON*i + 6) = -tmp_b;
		h_con(NOUTPUTCON*i + 7) = KAPPA_MAX;
	}



	/* difference matrix, need to change when more than 1 input */
	//MatrixXd C = Matrix<double, NP, NP>::Identity();
	//MatrixXd C1 = Matrix<double, NP, NP>::Zero();
	//C1.block<NP - 1, NP - 1>(1, 0) = -Matrix<double, NP - 1, NP - 1>::Identity();
	//C += C1;

	/* f, b determination */

//	VectorXd w = VectorXd::Ones(cols) * theta_r;            // curved road considered later.
	VectorXd w = VectorXd::Zero(NP);

	/* set model objective */
	f = RowVectorXd::Zero(cols);
	f.head(NC) = (Phi * planner_z + Ew * w).transpose() * Q * Gamma;  // vector uses head(), tail(), segment(p, q).
	f(NC) = P_Planner.WeightQ[3];

	double *c = f.data();
	GRBQuadExpr obj_f = 0;
	for (i = 0; i < cols; i++)
	{
		if (c[i] != 0)	obj_f += c[i] * vars[i];
	}
	model->setObjective(obj_h + obj_f);

	/* set model constraints(rhs) */
	VectorXd b_dummy_tmp(4);
	b_dummy_tmp << -NETA, -NETA, -NETA, 0;
	VectorXd b_dummy = b_dummy_tmp.replicate(2 * NP, 1);
	b = h_con - G_con * Phi*planner_z - (E_con + G_con * Ew)*w + b_dummy;

	if (model->getConstrs() == nullptr)
	{
		for (int i = 0; i < lhs_vec.size(); i++)
		{
			GRBConstr constr = model->addConstr(lhs_vec[i], '<', b(i));
			constraints.push_back(constr);
		}
	}
	else
	{
		for (int i = 0; i < constraints.size(); i++)
		{
			constraints[i].set(GRB_DoubleAttr_RHS, b(i));
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

			planner_z = z_ph.head(NSTATE);

			Path future_path;

			for (int i = 0; i < NP / 2; i++)
			{
				Point future_waypoint = offset2Point(z_ph(NSTATE*i), z_ph(NSTATE*i + 1), predictPoint[i]);
				future_path.push_back(future_waypoint, z_ph(NSTATE*i + 2), currentS + (i + 1) * vx*Tp);
			}


			pMutex->lock();
			planned_path = past_path + future_path;
			pMutex->unlock();


		}
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

	// update currentS
	currentS += vx * Tp;
}


void Planner_mpc::threadStep(CAR_STATE * p_DS_to_C, Path & planned_path)
{
	long frameCount = 0;
	int key = 0;
	while (!terminate)
	{
		long cFrameCount = p_DS_to_C->lTotalFrame;
		if (cFrameCount>=frameCount + 12)
		{
			frameCount = cFrameCount;
			step(p_DS_to_C, planned_path);
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
