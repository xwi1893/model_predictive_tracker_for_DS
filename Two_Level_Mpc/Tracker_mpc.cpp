#include "Tracker_mpc.h"

P_vehicle Tracker_mpc::Car{
	//Mass
	1100,

	//Inertia Z
	2942,

	//lf
	1.0,

	//lr
	1.635,

	//Cf
	6.3713e04,

	//Cr
	7.2995e04,

	//length
	4.4,

	//width
	1.725,

	//Kt
	17,

	//ar_max
	0.0271,                            // ar_max = ar_max - delta_r, delta_r = 0.014.

	//gamma_max
	0.4379
};

P_controller Tracker_mpc::P_TRACKER{
	// Weight Q
	{0,0,1.08,5.42},

	// Weight Ru
	{0.1},

	// Weight Rdu
	{10},

	// Weight dummy
	{1e06, 1e03},

	// Maximum input, 0.35 rad when steering angle, 2.6 kN when front tyre force.
	0.35,

	// Maximum input rate, 0.02 rad/step when steering angle, 0.2 kN/step when front tyre force.
	0.02
};


Tracker_mpc::Tracker_mpc(GRBEnv *tracker_env)
{
	env = tracker_env;
	model = new GRBModel(*env);
	
	obj_h = 0;
	u_next[0] = 0;
	terminate = false;

	ifstream frontTyreStream(FrontTyreLookup);
	size_t ft_index = 0;
	while (frontTyreStream && ft_index < num_fyf)
	{
		string s;
		if (!getline(frontTyreStream, s)) break;
		istringstream ss(s);
		vector<double> data;
		while (ss)
		{
			string str_data;
			if (!getline(ss, str_data, ',')) break;
			data.push_back(stod(str_data));
		}
		fyf[ft_index] = data[0];
		slipA[ft_index] = data[1];
		ft_index++;
	}
}

Tracker_mpc::~Tracker_mpc()
{
	delete model;
	model = nullptr;
	if (vars != nullptr)
	{
		delete vars;
		vars = nullptr;
	}
}

void Tracker_mpc::initialize()
{
	vx = 60 / 3.6;
	trackSteps = 0;

	RowVectorXd WeightR(1), WeightRdu(1), WeightQ(NSTATE);
	for (size_t i = 0; i < P_TRACKER.WeightQ.size(); i++)
	{
		WeightQ(i) = P_TRACKER.WeightQ[i];
	}
	WeightR << P_TRACKER.WeightR[0];
	WeightRdu << P_TRACKER.WeightRdu[0];
	RowVectorXd WQ, WRu, WRdu;
	WQ = WeightQ.replicate(1, NP);
	WRu = WeightR.replicate(1, NC);
	WRdu = WeightRdu.replicate(1, NC);

	Q = WQ.asDiagonal();
	Ru = WRu.asDiagonal();
	Rdu = WRdu.asDiagonal();
	
	MatrixXd As(NSTATE, NSTATE), Ad(NSTATE, NSTATE), Bs(NSTATE, NINPUT), Bd(NSTATE, NINPUT), Es(NSTATE, 1), Ed(NSTATE, 1);
	As << -2 * (Car.CF + Car.CR) / (Car.MASS*vx), 2 * (Car.CR*Car.LRAXIS - Car.CF*Car.LFAXIS) / (Car.MASS*vx*vx) - 1, 0, 0,
		2 * (Car.CR*Car.LRAXIS - Car.CF*Car.LFAXIS) / Car.IZ, -2 * (Car.CF*Car.LFAXIS*Car.LFAXIS + Car.CR*Car.LRAXIS*Car.LRAXIS) / (Car.IZ*vx), 0, 0,
		0, 1, 0, 0,
		vx, 0, vx, 0;
	Bs << 2.64 * Car.CF / (Car.MASS*vx), 2.64 * Car.CF*Car.LFAXIS / Car.IZ, 0, 0;
	Es << 0, 0, -vx, 0;

	//As << -2 * Car.CR / (Car.MASS*vx), 2 * Car.CR*Car.LRAXIS / (Car.MASS*vx*vx) - 1, 0, 0,
	//	2 * Car.CR*Car.LRAXIS / Car.IZ, -2 * Car.CR*Car.LRAXIS*Car.LRAXIS / (Car.IZ*vx), 0, 0,
	//	0, 1, 0, 0,
	//	vx, 0, vx, 0;
	//Bs << 2000 / (Car.MASS*vx), 2000 * Car.LFAXIS / Car.IZ, 0, 0;
	//Es << 0, 0, -vx, 0;


	/* continuous to discrete */
	int K = As.cols() + Bs.cols() + Es.cols();
	MatrixXd S = MatrixXd::Zero(K, K);
	S.block(0, 0, As.rows(), As.cols()) = As.block(0, 0, As.rows(), As.cols());
	S.block(0, As.cols(), Bs.rows(), Bs.cols()) = Bs.block(0, 0, Bs.rows(), Bs.cols());
	S.block(0, As.cols() + Bs.cols(), Es.rows(), Es.cols()) = Es.block(0, 0, Es.rows(), Es.cols());
	S *= Tc;
	S = S.exp();
	Ad = S.block(0, 0, As.rows(), As.cols());
	Bd = S.block(0, As.cols(), Bs.rows(), Bs.cols());
	Ed = S.block(0, As.cols() + Bs.cols(), Es.rows(), Es.cols());

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
		if (i<NC) Gamma_tmp << MatrixXd::Zero(NSTATE, i), Bd, MatrixXd::Zero(NSTATE, NC - i - 1);
		else Gamma_tmp << MatrixXd::Zero(NSTATE, NC);
		Gamma_part = Ad * Gamma_part + Gamma_tmp;
		Gamma.block<NSTATE, NC>(NSTATE*i, 0) = Gamma_part;
	}

	MatrixXd GPE(NSTATE*NP, NC + NSTATE + NP);
	GPE << Gamma, Phi, Ew;

	/* difference matrix, need to change when more than 1 input */
	MatrixXd C = Matrix<double, NC, NC>::Identity();
	MatrixXd C1 = Matrix<double, NC, NC>::Zero();
	C1.block<NC - 1, NC - 1>(1, 0) = -Matrix<double, NC - 1, NC - 1>::Identity();
	C += C1;

	/* H, A determination */
	MatrixXd H = MatrixXd::Zero(cols, cols);
	H.block<NINPUT*NC, NINPUT*NC>(0,0) = Gamma.transpose()*Q*Gamma + Ru + C.transpose()*Rdu*C;

	MatrixXd G_sta(4, NSTATE);
	G_sta << 1, -Car.LRAXIS / vx, 0, 0,
		-1, Car.LRAXIS / vx, 0, 0,
		0, 1, 0, 0,
		0, -1, 0, 0;                                  // |beta - lr*gamma/vx| < ARMAX, |gamma| < GAMMAMAX.
	MatrixXd G_con_sta = MatrixXd::Zero(4 * NP, NSTATE * NP);
	nblkdiag(G_sta, G_con_sta, NP);
	MatrixXd A = MatrixXd::Zero(4 * NP, cols);
	A.block<4 * NP, NC>(0, 0) = G_con_sta * Gamma;
	G_con_sta_rhs = G_con_sta * GPE.block<NSTATE*NP, NSTATE + NP>(0, NC);


	double q[cols][cols], a[rows][cols], lb[cols], ub[cols];

	for (size_t i = 0; i < NC; i++)
	{
		lb[i] = -P_TRACKER.UMAX;
		ub[i] = P_TRACKER.UMAX;
	}
	lb[NC] = 0;
	ub[NC] = GRB_INFINITY;
	lb[NC + 1] = 0;
	ub[NC + 1] = GRB_INFINITY;

	vars = model->addVars(lb, ub, NULL, NULL, NULL, cols);

	Map<Matrix<double, cols, cols, RowMajor>>(&q[0][0], cols, cols) = H;
	Map<Matrix<double, rows, cols, RowMajor>>(&a[0][0], rows, cols) = A;

	int i, j;
	for (i = 0; i < cols; i++)
		for (j = 0; j < cols; j++)
			if (q[i][j] != 0)
				obj_h += q[i][j] * vars[i] * vars[j];
	obj_h += P_TRACKER.WeightDummy[0] * vars[NC] + P_TRACKER.WeightDummy[1] * vars[NC + 1];

	for (i = 0; i < NC - 1; i++)
	{
		GRBLinExpr lhs = 0;
		lhs += vars[i] - vars[i + 1];
		model->addConstr(lhs, '<', P_TRACKER.DUMAX);
		model->addConstr(lhs, '>', -P_TRACKER.DUMAX);
	}

	/* construct stability constraints lhs */
	for (i = 0; i < rows; i++)
	{
		GRBLinExpr lhs = 0;
		for (j = 0; j < cols; j++)
		{
			if (a[i][j] != 0) lhs += a[i][j] * vars[j];
		}
		lhs -= vars[cols - 1];
		GRBConstr constr = model->addConstr(lhs, '<', 0);
		sta_con.push_back(constr);
	}

	/* construct environment constraints lhs */
	MatrixXd G_env(3, NSTATE);
	G_con_env_rhs = MatrixXd::Zero(3 * NP, NSTATE + NP);
	G_env << 0, 0, -Car.LRAXIS, 1,
		0, 0, 0, 1,
		0, 0, Car.LFAXIS, 1;
	RowVectorXd g_con_env(NC + NSTATE + NP);
	for (j = 0; j < 3; j++)
	{
		for (i = 0; i < NP; i++)
		{
			RowVectorXd G_env_row(NSTATE*NP);
			G_env_row << RowVectorXd::Zero(NSTATE*i), G_env.row(j), RowVectorXd::Zero(NSTATE*(NP - i - 1));
			g_con_env = G_env_row * GPE;
			GRBLinExpr lhs = 0;
			for (int k = 0; k < NC; k++)
			{
				if (g_con_env(k) != 0) lhs += g_con_env(k)*vars[k];
			}
			genv_lhs_vec.push_back(lhs);
			G_con_env_rhs.row(j*NP + i) = g_con_env.tail(NSTATE + NP);
		}
	}
}

int Tracker_mpc::receiveFromPlanner(MPC & planner)
{
	if (!planner.leftBound.empty() | !planner.rightBound.empty())
	{
		leftBound.assign(planner.leftBound.begin(), planner.leftBound.end());
		rightBound.assign(planner.rightBound.begin(), planner.rightBound.end());
		return 1;
	}
	return 0;
}


void Tracker_mpc::updateModel(CAR_STATE *currentState, Path &planned_path)
{
	Point curPos(currentState->dPos[0], currentState->dPos[1]);
	double d_offset = 0;
	double phi_offset = 0;
	double beginStateKappa = 0;
	int sgn = 1;

	int i, j;

// load planned_path to ref_path
	unique_lock<mutex> sbguard(planned_path.pathMutex, try_to_lock);
	if (sbguard.owns_lock())
	{
		if (!planned_path.isCopied)
		{
			ref_path = planned_path;
			planned_path.isCopied = true;
			s_index = 0;
			trackSteps = 0;
		}
		sbguard.unlock();
	}


	if (ref_path.size() > 0)
	{
		// d_offset to reference path
		while (s_index <= ref_path.size() - 1)
		{
			if (s_index == 0) s_index++;
			Vector2d u = ref_path.waypoints[s_index] - ref_path.waypoints[s_index - 1];
			Vector2d v = curPos - ref_path.waypoints[s_index];

			double angle = acos(u.dot(v) / (u.norm()*v.norm() + EPS));
			if (angle < PI / 2) s_index++;
			else
			{
				sgn = (curPos.orientation(ref_path.waypoints[s_index - 1]) >= ref_path.waypoints[s_index].orientation(ref_path.waypoints[s_index - 1])) ? 1 : -1;
				d_offset = v.norm()*sin(angle)*sgn;
				currentS = ref_path.s[s_index] + v.norm()*cos(angle);
				phi_offset = currentState->dAng[2] - ref_path.waypoints[s_index].orientation(ref_path.waypoints[s_index - 1]);
				break;
			}
		}

		bool isRestart = false;
		if ((!isObsDetected) && currentState->cObsDisp == 1) {
			isRestart = true;
			isObsDetected = true;
		}

		if (isRestart)
		{
			Point beginPoint = Point(currentState->dPos[0], currentState->dPos[1], currentState->dAng[2]);
			sbguard.lock();
			planned_path.beginState.first = beginPoint;
			planned_path.beginState.second = beginStateKappa;
			planned_path.isNeedReplan = true;
			planned_path.replanCv.notify_all();
			sbguard.unlock();
		}
		else if (++trackSteps >= 40)
		{
			Point beginPoint = ref_path.linear_interpolation(currentS, s_index);
			sbguard.lock();
			planned_path.beginState.first = beginPoint;
			planned_path.beginState.second = beginStateKappa;
			planned_path.isNeedReplan = true;
			planned_path.replanCv.notify_all();
			sbguard.unlock();
			trackSteps = 30;
		}
	}


	/* Predict S, x, y in prediction horizon */
	vector<double> predictS, predictS_rear, predictS_front, predictKappa;
	vector<Point> predictPoint, predictPoint_rear, predictPoint_front;
	for (i = 1; i <= NP; i++) {
		predictS_rear.emplace_back(currentS + vx * i*Tc - Car.LENGTH / 3.0);
		predictS.emplace_back(currentS + vx * i*Tc);
		predictS_front.emplace_back(currentS + vx * i*Tc + Car.LENGTH / 3.0);
	}

	if (ref_path.size() > 0)
	{
		ref_path.linear_interpolation(predictS_rear, predictPoint_rear);
		ref_path.linear_interpolation(predictS, predictPoint);
		ref_path.linear_interpolation(predictS, predictKappa);
		ref_path.linear_interpolation(predictS_front, predictPoint_front);
	}


	/* difference matrix, need to change when more than 1 input */
	MatrixXd C = Matrix<double, NC, NC>::Identity();
	MatrixXd C1 = Matrix<double, NC, NC>::Zero();
	C1.block<NC - 1, NC - 1>(1, 0) = -Matrix<double, NC - 1, NC - 1>::Identity();
	C += C1;

	/* H, f, A, b determination */
	VectorXd z(NSTATE);
	z << currentState->dBeta, currentState->yawRate, phi_offset, d_offset;              // column vector is default.
	VectorXd u_pre = VectorXd::Zero(NC);
	VectorXd w = VectorXd::Zero(NP);
	if (!predictKappa.empty())
	{
		for (i = 0; i < NP; i++) w(i) = predictKappa[i];
	}

	u_pre(0) = u_next[0];
	f = (Phi * z + Ew * w).transpose() * Q * Gamma - u_pre.transpose()*Rdu*C;

	//b = h_con - G_con * (Phi*z + Ew*w);
	VectorXd zw(NSTATE + NP);
	zw << z, w;
	VectorXd h_sta_rhs = (VectorXd(4) << Car.ARMAX, Car.ARMAX, Car.GAMMAMAX, Car.GAMMAMAX).finished().replicate(NP, 1);
	h_sta_rhs = h_sta_rhs - G_con_sta_rhs * zw;
	VectorXd h_env_rhs = -G_con_env_rhs * zw;

	double *c = f.data();
	GRBQuadExpr obj_f = 0;
	for (i = 0; i < NC; i++)
	{
		obj_f += c[i] * vars[i];
	}

	model->addConstr(vars[0], '<', u_pre(0) + P_TRACKER.DUMAX, "next_input_ub");
	model->addConstr(vars[0], '>', u_pre(0) - P_TRACKER.DUMAX, "next_input_lb");
	model->setObjective(obj_h + obj_f);

	/* add stability constratins to model */
	//for (size_t i = 0; i < sta_con.size(); i++)
	//{
	//	sta_con[i].set(GRB_DoubleAttr_RHS, h_sta_rhs(i));
	//}

	 
	/* add environment constraints to model */
	if (isObsDetected)
	{
		double h_tmp;
		auto &dummy_v = vars[cols - 1];
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
			for (i = 0; i < predictPoint_ptr->size(); i++) {
				Point &point = predictPoint_ptr->at(i);
				if (point.transform2b(h_tmp, rightBound)) {
					GRBConstr constr_env = model->addConstr(genv_lhs_vec[j*NP + i] - dummy_v, '<', h_tmp + h_env_rhs(j*NP + i));
					env_con.push_back(constr_env);
				}
				if (point.transform2b(h_tmp, leftBound)) {
					GRBConstr constr_env = model->addConstr(genv_lhs_vec[j*NP + i] + dummy_v, '>', h_tmp + h_env_rhs(j*NP + i));
					env_con.push_back(constr_env);
				}
			}
		}
	}
}

void Tracker_mpc::step()
{
	try
	{
		double objval;

		for (size_t i = 0; i < cols - 1; i++)
		{
			vars[i].set(GRB_DoubleAttr_Start, sol[i + 1]);
		}

		model->optimize();
		GRBQuadExpr thisObj = model->getObjective();
		if (model->get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
			objval = model->get(GRB_DoubleAttr_ObjVal);
			for (int i = 0; i < cols; i++)
			{
				sol[i] = vars[i].get(GRB_DoubleAttr_X);
			}
			u_next[0] = sol[0];   
			getOptimal.push_back(true);
		}
		else
		{
			getOptimal.push_back(false);
		}
		model->remove(model->getConstrByName("next_input_ub"));
		model->remove(model->getConstrByName("next_input_lb"));
		for (auto &constr : env_con)
		{
			model->remove(constr);
		}
		env_con.clear();
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

void Tracker_mpc::threadStep(CAR_STATE * p_DS_to_C, float * p_C_to_DS, Path &planned_path)
{
	long frameCount = 0;
	int key = 0;
	int trackSteps = 0;
	clock_t t_start;
	while (!terminate)
	{
		long cFrameCount = p_DS_to_C->lTotalFrame;
		if (cFrameCount >= frameCount + 3)
		{
			frameCount = cFrameCount;
			t_start = clock();
			updateModel(p_DS_to_C, planned_path);
			step();
			float next = (float)u_next[0]* RAD2DEG*Car.KT;
			//float next = (p_DS_to_C->dBeta + p_DS_to_C->yawRate * Car.LFAXIS / vx - force2slipA(u_next[0])) / 1.32;
			//next = next * Car.KT*RAD2DEG;
			*p_C_to_DS = next;
			tracker_Output.push_back(next);
			//tracker_timeCost.emplace_back((clock() - t_start)*1.0 / CLOCKS_PER_SEC * 1000);
			//cout << "The next steering angle is " << *p_C_to_DS << endl;
		}

		if (_kbhit() != 0) key = _getch();
		if (key == 'q') terminate = true;
	}
	planned_path.pathMutex.lock();
	planned_path.isNeedReplan = true;
	planned_path.replanCv.notify_all();
	planned_path.pathMutex.unlock();
}

double * Tracker_mpc::getOutput()
{
	return u_next;
}

//float Tracker_mpc::force2slipA(double force)
//{
//	float res_angle;
//	if (force < fyf[0]) res_angle = slipA[0];
//	else if (force > fyf[num_fyf - 1]) res_angle = slipA[num_fyf - 1];
//	else
//	{
//		int i = 0;
//		int j = num_fyf - 1;
//		while (j-i > 1)
//		{
//			int i_tmp = (i + j) / 2;
//			if (fyf[i_tmp] <= force) i = i_tmp;
//			else j = i_tmp;
//		}
//		res_angle = (fyf[j] - force) / (fyf[j] - fyf[i])*slipA[i] + (force - fyf[i]) / (fyf[j] - fyf[i])*slipA[j];
//	}
//	return res_angle * DEG2RAD;
//}
