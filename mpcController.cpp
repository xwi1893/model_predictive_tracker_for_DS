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

void MpcController::initialize()
{
	int ref_flag = loadReference("path.csv");
	if (ref_flag)
	{
		throw "Failed to load reference path!";
	}
	kappa = -1 / 200;
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

	A = MatrixXd::Zero(rows, cols);
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
	MatrixXd Temp = MatrixXd::Zero(NSTATE * NP + NSTATE, 1);
	MatrixXd Theta = MatrixXd::Zero(NSTATE * NP, NSTATE * NP);
	//Phi = MatrixXd::Zero(NSTATE * NP, NSTATE);
	//Gamma = MatrixXd::Zero(NSTATE * NP, NP);
	//Ew = MatrixXd::Zero(NSTATE * NP, 1);

	for (int i = 0; i <= NP; i++)
	{
		Temp.block<NSTATE, NSTATE>(NSTATE * i, 0) = Ad.pow(i).block<NSTATE, NSTATE>(0, 0);            // iterative product cost less time?
	}
	Phi = Temp.block<NSTATE * NP, NSTATE>(NSTATE, 0);

	for (int i = 0; i < NP; i++)
	{
		Theta.block(NSTATE * i, NSTATE * i, NSTATE * (NP - i), NSTATE) = Temp.block(0, 0, NSTATE * (NP - i), NSTATE);
	}

	MatrixXd Bd_tmp = MatrixXd::Zero(NSTATE * NP, NP);

	if (nblkdiag(Bd, Bd_tmp, NP))
	{
		Gamma = Theta * Bd_tmp;
		Ew = Theta * Ed.replicate(NP, 1);
	}
	else
	{
		throw "The size of B_tmp is incorrect!";
	}

	/* difference matrix, need to change when more than 1 input */
	MatrixXd C = Matrix<double, NP, NP>::Identity();
	MatrixXd C1 = Matrix<double, NP, NP>::Zero();
	C1.block<NP - 1, NP - 1>(1, 0) = -Matrix<double, NP - 1, NP - 1>::Identity();
	C += C1;

	/* H, f, A, b determination */
	H = Gamma.transpose()*Q*Gamma + Ru + C.transpose()*Rdu*C;
	A << C, -C;
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
			d_offset = v.norm()*sin(angle);
			break;
		}
	}

	if (s_index == ref_path.size()) throw "reference scope is exceeded!";

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
	VectorXd u_pre = VectorXd::Zero(cols);
	VectorXd dist = VectorXd::Ones(cols) * kappa;
	u_pre(0) = currentState->fStrBeta*DEG2RAD;
	f = (Phi * z + Ew * dist).transpose() * Q * Gamma - u_pre.transpose()*Rdu*C;
	VectorXd b_tmp;
	b_tmp << u_pre, -u_pre;
	b = VectorXd::Ones(rows)*DUMAX + b_tmp;
}

void MpcController::step()
{
	try
	{
		double *c = f.data();
		double *rhs = b.data();
		double q[cols][cols], a[rows][cols], lb[cols], ub[cols], objval;
		char sense[rows];
		for (size_t i = 0; i < cols; i++)
		{
			lb[i] = -UMAX;
			ub[i] = UMAX;
		}
		for (size_t i = 0; i < rows; i++)
		{
			sense[i] = '<';
		}
		bool success;

		Map<Matrix<double, cols, cols, RowMajor>>(&q[0][0], cols, cols) = H;
		Map<Matrix<double, rows, cols, RowMajor>>(&a[0][0], rows, cols) = A;

		success = this->dense_optimization(c, &q[0][0], &a[0][0], sense, rhs, lb, ub, NULL, sol, &objval);
		cout << "solution if found" << endl;
		//for (size_t i = 0; i < NP; i++) cout << sol[i] << endl;
		u_next[0] = sol[0];
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
}

MpcController::~MpcController()
{
	delete env;
}

bool MpcController::dense_optimization(double * c, double * q, double * a, char * sense, double * rhs, double * lb, double * ub, char * vtype, double * solution, double * objvalP)
{
	GRBModel model = GRBModel(*env);
	int i, j;
	bool success = false;

	/* Add variables to the model */

	GRBVar* vars = model.addVars(lb, ub, NULL, vtype, NULL, cols);

	/* Populate A matrix */

	for (i = 0; i < rows; i++) {
		GRBLinExpr lhs = 0;
		for (j = 0; j < cols; j++)
			if (a[i*cols + j] != 0)
				lhs += a[i*cols + j] * vars[j];
		model.addConstr(lhs, sense[i], rhs[i]);
	}

	GRBQuadExpr obj = 0;

	for (j = 0; j < cols; j++)
		obj += c[j] * vars[j];
	for (i = 0; i < cols; i++)
		for (j = 0; j < cols; j++)
			if (q[i*cols + j] != 0)
				obj += q[i*cols + j] * vars[i] * vars[j];

	model.setObjective(obj);

	model.optimize();

	if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
		*objvalP = model.get(GRB_DoubleAttr_ObjVal);
		for (i = 0; i < cols; i++)
			solution[i] = vars[i].get(GRB_DoubleAttr_X);
		success = true;
	}

	delete[] vars;

	return success;
}

double* MpcController::getOutput()
{
	return u_next;
}
