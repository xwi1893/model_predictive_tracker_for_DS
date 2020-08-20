#include <Windows.h>
#include <time.h>
#include "tchar.h"
#include "Planner_mpc.h"
#include "Tracker_mpc.h"

int main(int argc, char *argv[])
{
	/* share memory */
	auto size = 512;                      // sizeof(double)*6 ?
	HANDLE hSharedMemory_DS_to_C;
	hSharedMemory_DS_to_C = OpenFileMapping(
		FILE_MAP_ALL_ACCESS,   // read/write access
		FALSE,                 // do not inherit the name
		"p_DS_to_C");               // name of mapping object

	if (hSharedMemory_DS_to_C == NULL)
	{
		_tprintf(TEXT("Could not open file mapping object (%d).\n"), GetLastError());
		getchar();
		return -1;
	}

	auto p_DS_to_C = (CAR_STATE*)MapViewOfFile(hSharedMemory_DS_to_C, FILE_MAP_ALL_ACCESS, NULL, NULL, size);

	HANDLE hSharedMemory_C_to_DS;
	hSharedMemory_C_to_DS = OpenFileMapping(
		FILE_MAP_ALL_ACCESS,   // read/write access
		FALSE,                 // do not inherit the name
		"p_C_to_DS");               // name of mapping object

	if (hSharedMemory_C_to_DS == NULL)
	{
		_tprintf(TEXT("Could not open file mapping object (%d).\n"), GetLastError());
		getchar();
		return -1;
	}

	auto p_C_to_DS = (float*)MapViewOfFile(hSharedMemory_C_to_DS, FILE_MAP_ALL_ACCESS, NULL, NULL, size);

	/* Planner & controller initialize */
	GRBEnv *env = new GRBEnv();
	Path planned_path;
	vector<Point> obs_bound;
	{
		Point p1(-9840, 9.0);
		Point p2(-9830, 8.5);
		Point p3(-9827, 8.5);
		Point p4(-9820, 9.0);
		vector<Point> tmp{ p1, p2, p3, p4 };
		obs_bound = tmp;
	}

	mutex my_mutex;
	Planner_mpc planner(env, &my_mutex);
	planner.initialize(60 / 3.6, obs_bound);
	Tracker_mpc tracker(env, &my_mutex);
	tracker.initialize();
	tracker.receiveFromPlanner(planner);

	cout << "press any key to continue..." << endl;
	_getch();

	/* planner & tracker thread */
	thread pThread(&Planner_mpc::threadStep, ref(planner), p_DS_to_C, ref(planned_path));
	thread tThread(&Tracker_mpc::thread_step, ref(tracker), p_DS_to_C, p_C_to_DS, ref(planned_path));

	pThread.join();
	tThread.join();

	/* release resources */
	delete env;
	env = nullptr;

	return 0;
}
