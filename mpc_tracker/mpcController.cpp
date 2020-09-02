#include "mpcController.h"


int nblkdiag(MatrixXd &M, MatrixXd &N, int n)
{
	if (N.rows() != M.rows()*n || N.cols() != M.cols()*n) return 0;
	for (int i = 0; i < n; i++)
	{
		N.block(i*M.rows(), i*M.cols(), M.rows(), M.cols()) = M.block(0, 0, M.rows(), M.cols());
	}
	return 1;
}

void readState(ifstream &source, vector<CAR_STATE> &states)
{
	while (source)
	{
		string s;
		if (!getline(source, s)) break;
		istringstream ss(s);
		vector<string> line;
		while (ss)
		{
			string str_data;
			if (!getline(ss, str_data, ',')) break;
			line.emplace_back(str_data);
		}
		CAR_STATE state;
		state.lTotalFrame = stol(line[0]);
		state.dtime = stod(line[1]);
		state.fSpeed = stof(line[2]);
		state.dPos[0] = stod(line[3]);
		state.dPos[1] = stod(line[4]);
		state.dPos[2] = stod(line[5]);
		state.dAng[0] = stod(line[6]);
		state.dAng[1] = stod(line[7]);
		state.dAng[2] = stod(line[8]);
		state.yawRate = stod(line[9]);
		state.dBeta = stod(line[10]);
		state.fStrBeta = stod(line[11]);
		states.push_back(state);
	}
}

void MpcController::initialize()
{
	int ref_flag = loadReference("path.csv");
	if (ref_flag)
	{
		throw "Failed to load reference path!";
	}
	kappa = 1 / 201.5;
	Vx = 60 / 3.6;

	RowVectorXd WeightRu(1), WeightRdu(1);
	RowVectorXd WeightQ = Map<VectorXd>(P_MPC.WeightQ, NSTATE, 1);
	WeightRu << P_MPC.WeightRu;
	WeightRdu << P_MPC.WeightRdu;
	RowVectorXd WQ, WRu, WRdu;
	WQ = WeightQ.replicate(1, NP);
	WRu = WeightRu.replicate(1, NP);
	WRdu = WeightRdu.replicate(1, NP);

	Q = WQ.asDiagonal();
	Ru = WRu.asDiagonal();
	Rdu = WRdu.asDiagonal();

	MatrixXd As(NSTATE, NSTATE), Ad(NSTATE, NSTATE), Bs(NSTATE, 1), Bd(NSTATE, 1), Es(NSTATE, 1), Ed(NSTATE, 1);
	As << -2*(CClassHatchback.CF + CClassHatchback.CR) / (CClassHatchback.MASS*Vx), 2*(CClassHatchback.CR*CClassHatchback.LRAXIS - CClassHatchback.CF*CClassHatchback.LFAXIS) / (CClassHatchback.MASS*Vx*Vx) - 1, 0, 0,
		2*(CClassHatchback.CR*CClassHatchback.LRAXIS - CClassHatchback.CF*CClassHatchback.LFAXIS) / CClassHatchback.IZ, -2 * (CClassHatchback.CF*CClassHatchback.LFAXIS*CClassHatchback.LFAXIS + CClassHatchback.CR*CClassHatchback.LRAXIS*CClassHatchback.LRAXIS) / (CClassHatchback.IZ*Vx), 0, 0,
		0, 1, 0, 0,
		Vx, 0, Vx, 0;
	Bs << 2 * CClassHatchback.CF / (CClassHatchback.MASS*Vx), 2 * CClassHatchback.CF*CClassHatchback.LFAXIS / CClassHatchback.IZ, 0, 0;
	Es << 0, 0, -Vx, 0;


	/* continuous to discrete */
	auto K = As.cols() + Bs.cols() + Es.cols();
	MatrixXd S = MatrixXd::Zero(K, K);
	S.block(0, 0, As.rows(), As.cols()) = As.block(0, 0, As.rows(), As.cols());
	S.block(0, As.cols(), Bs.rows(), Bs.cols()) = Bs.block(0, 0, Bs.rows(), Bs.cols());
	S.block(0, As.cols() + Bs.cols(), Es.rows(), Es.cols()) = Es.block(0, 0, Es.rows(), Es.cols());
	S *= TS;
	S = S.exp();
	Ad = S.block(0, 0, As.rows(), As.cols());
	Bd = S.block(0, As.cols(), Bs.rows(), Bs.cols());
	Ed = S.block(0, As.cols() + Bs.cols(), Es.rows(), Es.cols());

	/* state elimination */
	Phi = MatrixXd::Zero(NSTATE * NP, NSTATE);
	Gamma = MatrixXd::Zero(NSTATE * NP, NP);
	Ew = MatrixXd::Zero(NSTATE * NP, NP);
	MatrixXd Phi_part = MatrixXd::Identity(NSTATE, NSTATE);
	MatrixXd Gamma_part = MatrixXd::Zero(NSTATE, NP);
	MatrixXd Ew_part = MatrixXd::Zero(NSTATE, NP);

	for (size_t i = 0; i < NP; i++)
	{
		Phi_part = Ad * Phi_part;
		Phi.block<NSTATE, NSTATE>(NSTATE*i, 0) = Phi_part;

		MatrixXd Ew_tmp = MatrixXd::Zero(NSTATE, NP);
		Ew_tmp.block<NSTATE, 1>(0, i) = Ed;
		Ew_part = Ad * Ew_part + Ew_tmp;
		Ew.block<NSTATE, NP>(NSTATE*i, 0) = Ew_part;

		MatrixXd Gamma_tmp(NSTATE, NP);
		Gamma_tmp << MatrixXd::Zero(NSTATE, i), Bd, MatrixXd::Zero(NSTATE, NP - i - 1);
		Gamma_part = Ad * Gamma_part + Gamma_tmp;
		Gamma.block<NSTATE, NP>(NSTATE*i, 0) = Gamma_part;
	}

	/* difference matrix, need to change when more than 1 input */
	MatrixXd C = Matrix<double, NP, NP>::Identity();
	MatrixXd C1 = Matrix<double, NP, NP>::Zero();
	C1.block<NP - 1, NP - 1>(1, 0) = -Matrix<double, NP - 1, NP - 1>::Identity();
	C += C1;

	/* H, f, A, b determination */
	MatrixXd H = Gamma.transpose()*Q*Gamma + Ru + C.transpose()*Rdu*C;
	//A = MatrixXd(rows, cols);
	//A << C, -C;

	double q[cols][cols], a[rows][cols], lb[cols], ub[cols];
	char sense[rows];

	for (size_t i = 0; i < cols; i++)
	{
		lb[i] = -UMAX;
		ub[i] = UMAX;
	}
	//for (size_t i = 0; i < rows; i++)
	//{
	//	sense[i] = '<';
	//}

	vars = model->addVars(lb, ub, NULL, NULL, NULL, cols);

	Map<Matrix<double, cols, cols, RowMajor>>(&q[0][0], cols, cols) = H;
	//Map<Matrix<double, rows, cols, RowMajor>>(&a[0][0], rows, cols) = A;
	
	int i, j;
	for (i = 0; i < cols; i++)
		for (j = 0; j < cols; j++)
			if (q[i*cols + j] != 0)
				obj_h += q[i][j] * vars[i] * vars[j];

	for (i = 0; i < cols-1; i++)
	{
		GRBLinExpr lhs = 0;
		lhs += vars[i] - vars[i + 1];
		model->addConstr(lhs, '<', DUMAX);
		model->addConstr(lhs, '>', -DUMAX);
	}
}

int MpcController::loadReference(const string fileName)
{
	ifstream ref_file(fileName);
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
		ref_path.push_back(Point(data[0], data[1]));
	}
	if (!ref_path.empty())
	{
		return 0;
	}
	else
	{
		return -1;
	}
}

void MpcController::updateModel(CAR_STATE *currentState, size_t &s_index)
{
	Point curPos(currentState->dPos[0], currentState->dPos[1]);
	double d_offset = 0;
	double phi_offset = 0;
	double v_lat = 0;
	int sgn = 1;

	// d_offset to reference path
	while (s_index <= ref_path.size() - 1)
	{
		if (s_index == 0) s_index++;
		Vector2d u = ref_path[s_index] - ref_path[s_index - 1];
		Vector2d v = curPos - ref_path[s_index];

		double angle = acos(u.dot(v) / (u.norm()*v.norm()));
		if (angle < PI / 2) s_index++;
		else
		{
			sgn = (curPos.orientation(ref_path[s_index - 1]) >= ref_path[s_index].orientation(ref_path[s_index - 1])) ? 1 : -1;
			d_offset = v.norm()*sin(angle)*sgn;
			break;
		}
	}


	// phi_offset to reference path
	double head_angle = currentState->dAng[2];
	phi_offset = head_angle - atan2(ref_path[s_index].y - ref_path[s_index - 1].y, ref_path[s_index].x - ref_path[s_index - 1].x);


	/* difference matrix, need to change when more than 1 input */
	MatrixXd C = Matrix<double, NP, NP>::Identity();
	MatrixXd C1 = Matrix<double, NP, NP>::Zero();
	C1.block<NP - 1, NP - 1>(1, 0) = -Matrix<double, NP - 1, NP - 1>::Identity();
	C += C1;

	/* H, f, A, b determination */
	VectorXd z(4);
	z << currentState->dBeta, currentState->yawRate, phi_offset, d_offset;

	VectorXd dist = VectorXd::Ones(cols) * kappa;
	VectorXd u_pre = VectorXd::Zero(cols);
	u_pre(0) = currentState->fStrBeta*DEG2RAD;
	f = (Phi * z + Ew * dist).transpose() * Q * Gamma - u_pre.transpose()*Rdu*C;
	double *c = f.data();
	GRBQuadExpr obj_f = 0;
	for (int i = 0; i < cols; i++)
	{
		obj_f += c[i] * vars[i];
	}

	model->addConstr(vars[0], '<', u_pre(0) + DUMAX, "next_input_ub");
	model->addConstr(vars[0], '>', u_pre(0) - DUMAX, "next_input_lb");
	GRBQuadExpr obj = obj_h + obj_f;
	model->setObjective(obj);
}

void MpcController::step()
{
	try
	{
		double objval;

		for (size_t i = 0; i < cols-1; i++)
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
		}
		model->remove(model->getConstrByName("next_input_ub"));
		model->remove(model->getConstrByName("next_input_lb"));
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

void MpcController::terminate()
{
}

MpcController::MpcController()
{
	env = new GRBEnv();
	model = new GRBModel(*env);
	obj_h = 0;
	fill_n(sol, cols, 0.0);
}

MpcController::~MpcController()
{
	delete env;
	delete model;
	env = nullptr;
	model = nullptr;
	if (vars != nullptr)
	{
		delete vars;
		vars = nullptr;
	}
}


double* MpcController::getOutput()
{
	return u_next;
}
