#include "tracker.h"

Tracker_mpc::Tracker_mpc()
{
	// set FILE struct
	fp = NULL;
	if (SET_PRINTLEVEL_MpcTracker > 0) {
		fp = fopen("tracker_info", "w+");
		if (fp == NULL) {
			throw "fopen did not work!";
		}
		rewind(fp);
	}
	
	// load path
	int flag = road_ref.loadFile("road.csv");
	if (!flag) throw "Failed to load reference path!"; 

	// initialize
	initialize();
}

Tracker_mpc::~Tracker_mpc()
{
	if (fp) fp = NULL;
}

void Tracker_mpc::initialize()
{
	vx = 16.66667;
	memset(params.H, 0, sizeof(params.H));
	vector<MpcTracker_float> H = { 10, 0, 0, 100, 1, 0.1 };
	for (int i = 0; i < H.size(); i++) {
		params.H[7 * i] = H[i];
	}

	// read AD
	MatrixXd AD(NSTATE + NINPUT, NSTATE + 2 * NINPUT);
	ifstream file;
	file.open("config/config.txt", ios::in);
//	cout << file.good() << endl;


	int i = 0;
	while (file)
	{
		string s;
		if (!getline(file, s)) break;
		istringstream ss(s);
		if (i < NSTATE + NINPUT) {
			for (size_t j = 0; j < NSTATE + 2 * NINPUT; j++) {
				string str_data;
				if (!getline(ss, str_data, ',')) break;
				AD(i, j) = stod(str_data);
			}
		}
		i++;
	}

	A = AD.block<NSTATE + NINPUT, NSTATE + NINPUT>(0, 0);
	D = AD.block<NSTATE + NINPUT, NINPUT>(0, NSTATE + NINPUT);

	//double* A_data = A.data();
	//for (int i = 0; i < 25;i++) cout << *A_data++ << ',';

	//double* D_data = D.data();
	//cout << endl;
	//for (int i = 0; i < 5; i++) cout << *D_data++ << ',';
}

solver_int32_default Tracker_mpc::step(CAR_STATE * currentState)
{
	solver_int32_default exit_code;

	double x = currentState->dPos[0], y = currentState->dPos[1];

	Waypoint location = road_ref.cart2frenet(x, y, 0, road_ref.size());
	double currentS = location.s;
	VectorXd xInit(NSTATE+NINPUT);
	xInit << currentState->dBeta, currentState->yawRate, currentState->yawAngle - location.theta, location.offset, u_next[0];

	double ref_kr, ref_ey;
	Waypoint p_int;
	interp<double, Waypoint>(road_ref.s, road_ref.waypoints, currentS, p_int);

	for (int i = 0; i < NP; i++) {
		ref_kr = p_int.kappa; ref_ey = p_int.offset;
		currentS = currentS + vx * Tc / (1 - ref_kr * ref_ey);
		double ud = ref_kr * vx / (1 - ref_kr * ref_ey);

		VectorXd eq_c;

		if (i == 0) {
			eq_c = -A * xInit - D * ud;
			memcpy(params.c + NSTATE * i, eq_c.data(), sizeof(eq_c));
		}
		else
		{
			eq_c = -D * ud;
			memcpy(params.c + NSTATE * i, eq_c.data(), sizeof(eq_c));
		}
		interp<double, Waypoint>(road_ref.s, road_ref.waypoints, currentS, p_int);
		/*
		double ref[NSTATE + NINPUT];
		memset(ref, 0, sizeof(ref));
		memcpy(params.Reference_Value + (NSTATE + NINPUT)*i, ref, sizeof(ref));
		*/
	}

	// reference always zero: tracking the lane
	memset(params.Reference_Value, 0, sizeof(params.Reference_Value));

	exit_code = MpcTracker_solve(&params, &output, &info, fp);
	if (exit_code == OPTIMAL_MpcTracker) {
		u_next[0] = output.u0[0];
	}
	else
	{
		cout << "The solver failed to get optimal solution!" << endl;
		u_next[0] = 0;
	}

	return exit_code;
}

double * Tracker_mpc::getOutput()
{
	return u_next;
}

void Tracker_mpc::threadStep(CAR_STATE * p_DS_to_C, float * p_C_to_DS, Path & planned_path)
{
	long frameCount = 0;
	int key = 0;
	clock_t t_start;
	while (!terminate)
	{
		long cFrameCount = p_DS_to_C->lTotalFrame;
		if (cFrameCount >= frameCount + 3)
		{
			frameCount = cFrameCount;
			t_start = clock();
			solver_int32_default exit_code = step(p_DS_to_C);
			if (exit_code == OPTIMAL_MpcTracker) {
				float next = (float)u_next[0] * RAD2DEG * Kt;
				*p_C_to_DS = next;
			}
			//tracker_timeCost.emplace_back((clock() - t_start)*1.0 / CLOCKS_PER_SEC * 1000);
			//cout << "The next steering angle is " << *p_C_to_DS << endl;
		}

		if (_kbhit() != 0) key = _getch();
		if (key == 'q') terminate = true;
	}
}
