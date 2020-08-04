#include <iostream>
#include <Windows.h>
#include <conio.h>
#include "tchar.h"
#include "mpcController.h"

int main(int argc, char *argv[])
{
	MpcController mpcController = MpcController();
	mpcController.initialize();

	/* share memory */
	auto size = 512;                      // sizeof(double)*6 ?
	HANDLE hSharedMemory_DS_to_C;
	hSharedMemory_DS_to_C = OpenFileMapping(
		FILE_MAP_ALL_ACCESS,   // read/write access
		FALSE,                 // do not inherit the name
		"p_DS_to_C");               // name of mapping object

	if (hSharedMemory_DS_to_C == NULL)
	{
		_tprintf(TEXT("Could not open file mapping object (%d).\n"),
			GetLastError());
		getchar();
		return 21;
	}

	auto p_DS_to_C = (CAR_STATE*)MapViewOfFile(hSharedMemory_DS_to_C, FILE_MAP_ALL_ACCESS, NULL, NULL, size);

	HANDLE hSharedMemory_C_to_DS;
	hSharedMemory_C_to_DS = OpenFileMapping(
		FILE_MAP_ALL_ACCESS,   // read/write access
		FALSE,                 // do not inherit the name
		"p_C_to_DS");               // name of mapping object

	if (hSharedMemory_C_to_DS == NULL)
	{
		_tprintf(TEXT("Could not open file mapping object (%d).\n"),
			GetLastError());
		getchar();
		return 31;
	}

	auto p_C_to_DS = (double*)MapViewOfFile(hSharedMemory_C_to_DS, FILE_MAP_ALL_ACCESS, NULL, NULL, size);

	/* read data and output control command */
	long frameCount = 0;
	size_t s_index = 0;
	bool Start = false;
	vector<double> control_command;
	cout << "press any key to continue..." << endl;
	if (_getch()) Start = true;

	while (Start)
	{
		long cFrameCount = p_DS_to_C->lTotalFrame;
		if (cFrameCount >= frameCount + 3)
		{
			frameCount = cFrameCount;
			mpcController.updateModel(p_DS_to_C, s_index);
			mpcController.step();
			double *next = mpcController.getOutput();
			*p_C_to_DS = next[0] * Kt * RAD2DEG;
			control_command.push_back(*p_C_to_DS);
			cout << "The next steering angle is " << *p_C_to_DS << endl;
		}
		int key;
		if (_kbhit() != 0) key = _getch();
		else key = 0;
		if (key == 'q') Start = false;
	}

	return 0;
}