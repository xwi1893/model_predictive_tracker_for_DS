#include <iostream>
#include <tchar.h>
#include <Windows.h>
#include <stdlib.h>
#include <stdio.h>
#include <conio.h>
#include <cstdlib>
#include <time.h>
#include "simulator.h"

using namespace std;

bool EndApp = false;

template<typename T>
T* createSharedMemory(HANDLE* hSharedMemory, LPCSTR name, HANDLE* mutex, LPCSTR mutexName) {
	int size = 512;
	*hSharedMemory = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, size, name);

	if (*hSharedMemory == NULL) {
		printf(TEXT("Could not create file mapping object (%d).\n"),GetLastError());
		return NULL;
	}

	T* ptr = (T*)MapViewOfFile(*hSharedMemory, FILE_MAP_ALL_ACCESS, NULL, NULL, size);

	*mutex = CreateMutex(NULL, FALSE, mutexName);
	if (*mutex == NULL) {
		printf(TEXT("Could not create mutex for file mapping (%d). \n"), GetLastError());
		return NULL;
	}
	return ptr;
}

template<typename T>
void closeSharedMemory(T* ptr, HANDLE* hSharedMemory, HANDLE* mutex) {
	UnmapViewOfFile(ptr);
	CloseHandle(*hSharedMemory);
	if (*mutex) CloseHandle(*mutex);
}

int main(int argc, char* argv) {
	double xInit = 6001.27, yInit = 2208.007;
	double speed = 16.66667;
	double yawAngle = 0, yawRate = 0;
	double Tp = 0.1;

	Simulator simulator(xInit, yInit, speed, yawAngle, yawRate, Tp);
	simulator.addObstacle(1, 1, 6179.877, 2210.001, PI/4, 0, 0);

	HANDLE hSharedMemory_DS_to_C = NULL, DS_to_C_Mutex = NULL;
	CAR_STATE* p_DS_to_C = createSharedMemory<CAR_STATE>(&hSharedMemory_DS_to_C, "p_DS_to_C", &DS_to_C_Mutex, "DS_to_C_Mutex");
	*p_DS_to_C = simulator.state;

	if (p_DS_to_C == NULL) {
		printf("Failed to create CAR_STATE ptr. \n");
		return 0;
	}

	HANDLE hSharedMemory_C_to_DS = NULL, C_to_DS_Mutex = NULL;
	float* p_C_to_DS = createSharedMemory<float>(&hSharedMemory_C_to_DS, "p_C_to_DS", &C_to_DS_Mutex, "C_to_DS_Mutex");
	if (p_C_to_DS == NULL) {
		printf("Failed to create float ptr. \n");
		return 0;
	}

	/* loop for simulator */
	clock_t prev_time;
	bool start = false;
	while (!EndApp)
	{
		system("pause");
		start = true;
		prev_time = clock();
		while (start) {
			//cout << "simulator is running..." << endl;
			if (_kbhit()) {
				int key = _getch();
				if (key == 'p') {
					system("pause");
					system("cls");
				}

				if (key == 's') start = false;
				if (key == 'q') {
					start = false;
					EndApp = true;
				}
			}
			clock_t current_time = clock();
			if (current_time - prev_time >= 100) {
				prev_time = current_time;

				WaitForSingleObject(C_to_DS_Mutex, INFINITE);
				float u = *p_C_to_DS;
				ReleaseMutex(C_to_DS_Mutex);

				simulator.update(u);
				cout << "simulator is updated." << "pos X is :" << simulator.state.dPos[0] <<
					", pos Y is :" << simulator.state.dPos[1] << endl;

				WaitForSingleObject(DS_to_C_Mutex, INFINITE);
				*p_DS_to_C = simulator.state;
				ReleaseMutex(DS_to_C_Mutex);
			}
		}
		system("cls");
	}

	closeSharedMemory(p_DS_to_C, &hSharedMemory_DS_to_C, &DS_to_C_Mutex);
	closeSharedMemory(p_C_to_DS, &hSharedMemory_C_to_DS, &C_to_DS_Mutex);

}