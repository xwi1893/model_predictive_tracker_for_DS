#include <time.h>
#include "tchar.h"
#include "planner.h"
#include "tracker.h"

int main(int argc, char* argv) {
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
	HANDLE DS_to_C_Mutex = OpenMutex(MUTEX_ALL_ACCESS, FALSE, "DS_to_C_Mutex");

	if (GetLastError() == ERROR_FILE_NOT_FOUND)
	{
		cout << "OpenMutex fail: mutex not found!" << endl;
		return -1;
	}

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
	HANDLE C_to_DS_Mutex = OpenMutex(MUTEX_ALL_ACCESS, FALSE, "C_to_DS_Mutex");

	if (GetLastError() == ERROR_FILE_NOT_FOUND)
	{
		cout << "OpenMutex fail: mutex not found!" << endl;
		return -1;
	}

	Tracker_mpc tracker;
	Planner_mpc planner;
	Environment env;

	Path planned_path;

	cout << "press any key to continue..." << endl;
	_getch();

	/* planner & tracker thread */
	thread eThread(&Environment::threadStep, ref(env), p_DS_to_C, ref(planned_path));
	thread pThread(&Planner_mpc::threadStep, ref(planner), p_DS_to_C, ref(planned_path), ref(env));
	thread tThread(&Tracker_mpc::threadStep, ref(tracker), p_DS_to_C, p_C_to_DS, ref(planned_path));

	eThread.join();
	pThread.join();
	tThread.join();

	if (DS_to_C_Mutex) CloseHandle(DS_to_C_Mutex);
	if (C_to_DS_Mutex) CloseHandle(C_to_DS_Mutex);

	return 0;
}
